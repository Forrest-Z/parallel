#include <Planner/Planner.h>
#include <Planner/tool/GuideDecider.h>
#include <Planner/impl/LatticePlanner.h>
#include <Planner/PlannerConfig.h>
#include <Planner/.PlannerModule.h>
#include <Planner/rule/StopLine.h>
#include <Planner/rule/Passable.h>

using namespace nox::app;
USING_NAMESPACE_NOX;
using std::endl;

void Planner::Initialize()
{
    _trajectoryStitcher = New<TrajectoryStitcher>(); // TODO: 参数初始化
    _algorithm = New<LatticePlanner>();

    InitializeDeciders();

    _scene_server.SetReceiver({"scene"});
    _trajectory_plotter.Advertise({"planner_plot"});

    cache::WriteEgoVehicle(AddressOf(_vehicle));

    Logger::GlobalLogLevel(Logger::Debug);
    analyze_global_enable(true);
}

void Planner::InitializeDeciders()
{
    auto guideDecider = New<GuideDecider>(AddressOf(_vehicle));
    guideDecider
        ->AddRule<rule::StopLine>()
        ->AddRule<rule::Passable>();

    _deciders.push_back(guideDecider);
}

void Planner::Process(nav_msgs::Odometry vehicle_state, optional<nox_msgs::Trajectory> &trajectory)
{
    _vehicle.From(vehicle_state);

    nox_msgs::GetSceneRequest request_for_scene;
    _vehicle.pose.To(request_for_scene.location.pose);
    request_for_scene.location.header = vehicle_state.header;

    _scene_server.SendRequest(request_for_scene); // TODO: 超时处理
    _scene.From(_scene_server.GetResponse().scene);

    _scene.Refresh({"scene"});
    Logger::D("Planner")
        << "Scene Updated !" << endl
        << "(x, y): " << _vehicle.pose.x << " " << _vehicle.pose.y << endl
        << "(v, w): " << _vehicle.v.x << " " << _vehicle.w.z << endl;

    Trajectory planning_trajectory;
    if(mailboxes.trajectory.SendHistoryCount() != 0)
        planning_trajectory.From(mailboxes.trajectory.LastSend());


    auto result = Process(planning_trajectory);

    if(result.Fail())
    {
        Logger::W("Planner") << result.Message();
    }
    else
    {
        Logger::I("Planner") << "Planning Successfully !";
        trajectory.emplace();
        planning_trajectory.To(trajectory.value());
        planning_trajectory.Refresh({"trajectory"});

        //region 规划结果接力测试
        while (false)
        {
            auto nearest_index = planning_trajectory.QueryNearestByPosition(_vehicle.pose.t);
            auto nearest_point = planning_trajectory[nearest_index];
            auto next_index    = planning_trajectory.QueryNearestByTime(nearest_point.t + 1);
            auto next_point    = planning_trajectory[next_index];

            auto result = Process(planning_trajectory);
            if(result.Fail())
            {
                Logger::W("Planner") << result.Message() << " [END LOOP]";
                break;
            }
            else
            {
                planning_trajectory.Refresh({"trajectory"});
            }
        }
        //endregion
    }

    //region 打印速度、加速度到rqt上
    if(false)
    {
        geometry_msgs::Point point;
        double last_t = 0;
        for(auto & i : planning_trajectory)
        {
            Clock::Sleep((i.t - last_t) * 1000);

            point.x = i.v;
            point.y = i.a;
            _trajectory_plotter.Send(point);

            last_t = i.t;
        }
    }
    //endregion
}


Planner::Result Planner::Plan(type::Trajectory &trajectory, bool enable_stitch)
{
    /// 1. 计算缝合轨迹，或裁短规划轨迹
    if(enable_stitch)
    {
        bool is_replan = false;
        trajectory = _trajectoryStitcher->FromLastTrajectoryByPosition(_vehicle, trajectory, &is_replan);
//    trajectory = _trajectoryStitcher->InitialTrajectory(*vehicle);
    }
    else
    {
        auto nearest_index = trajectory.QueryNearestByPosition(_vehicle.pose.t);
        auto nearest_point = trajectory[nearest_index];
        auto forward_index  = trajectory.QueryNearestByTime(nearest_point.t + _param._reserve._forward_time);
        auto backward_index = trajectory.QueryNearestByTime(nearest_point.t - _param._reserve._backward_time);

        trajectory = trajectory.SubTrajectory(backward_index, forward_index);
    }


    /// 2. 封装车道线为reference_line
    PlannerBase::Frame frame;
    frame.scene = AddressOf(_scene);
    frame.vehicle = AddressOf(_vehicle);

    for(auto & i : _scene.GuideLines)
    {
        frame.references.push_back(New<ReferenceLine>(*i.second));
    }

    /// 3. 运用决策器，决策信息放在ReferenceLine中
    for(auto & decider : _deciders)
    {
        decider->Execute(frame.references);
    }

    /// 5. 运用规划器
    auto planning_result = _algorithm->Plan(trajectory, frame, trajectory);

    /// 6. 返回处理结果
    if(planning_result.OK())
    {
        return Result(ErrorCode::Success);
    }
    else
    {
        return Result(ErrorCode::PlanningFail, "Unable to plan");
    }
}

Planner::Result Planner::Process(type::Trajectory &last_trajectory)
{
    Planner::Result result;
    auto vehicle = AddressOf(_vehicle);
    auto scene = AddressOf(_scene);

    if (last_trajectory.Empty())
        result = Plan(last_trajectory);
    else
    {
        auto nearest_index = last_trajectory.QueryNearestByPosition(_vehicle.pose.t);
        auto nearest_point = last_trajectory[nearest_index];

        if (_vehicle.pose.t.DistanceTo(nearest_point.pose.t) > _param._threshold._replan_distance)
            result = Plan(last_trajectory);
        else
        {
            auto last_point = last_trajectory.Back();
            PlannerBase::Frame frame;
            frame.scene = scene;
            frame.vehicle = vehicle;

            if(last_point.t - nearest_point.t < _param._threshold._replan_time)
                result = Plan(last_trajectory);
            else if(auto check_result = _algorithm->Check(last_trajectory, frame); check_result.Fail())
            {
                Logger::W("Planner") << "Check last trajectory failed. Because of " << PlannerBase::ParseErrorCode(check_result.Code());
                result = Plan(last_trajectory);
            }
            else if(last_point.t - nearest_point.t < _param._threshold._extend_time)
            {
                if(result = Plan(last_trajectory, false); result.Fail())
                {
                    Logger::W("Planner") << "Extend last trajectory failed";
                    result = Plan(last_trajectory);
                }
            }
        }
    }

    return result;
}



