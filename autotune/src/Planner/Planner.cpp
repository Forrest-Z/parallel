#include <Planner/Planner.h>
#include <Planner/impl/LatticePlanner.h>
#include <Planner/PlannerConfig.h>
#include <Planner/.PlannerModule.h>
#include <Planner/rule/StopLine.h>
#include <Planner/rule/Passable.h>
#include <nox>

using namespace nox::app;
USING_NAMESPACE_NOX;
using std::endl;

void Planner::Initialize()
{
    _trajectoryStitcher = New<TrajectoryStitcher>(); // TODO: 参数初始化
    _algorithm = New<LatticePlanner>();

    InitializeDeciders();
    InitializeParameters();

    _scene_server.SetReceiver({"scene"});
    _trajectory_plotter.Advertise({"planner_plot"});

    cache::WriteEgoVehicle(AddressOf(_vehicle));

    Logger::GlobalLogLevel(Logger::Debug);
    analyze_global_enable(true);
}

void Planner::InitializeDeciders()
{
    auto guideDecider = New<DecisionMaker>();
    guideDecider
        ->AddRule<rule::StopLine>()
        ->AddRule<rule::Passable>();

    _deciders.push_back(guideDecider);
}

void Planner::InitializeParameters()
{
    _vehicle.param.length.x = params.Vehicle.Physical.Length;
    _vehicle.param.length.y = params.Vehicle.Physical.Weight;
    _vehicle.param.baseSpeed = params.Vehicle.BaseSpeed / 3.6;
}

void Planner::Process(nav_msgs::Odometry vehicle_state, optional<nox_msgs::Trajectory> &trajectory)
{
    static system::Timer timer;


    //region 更新车的状态以及规划场景
    timer.Start();
    _vehicle.From(vehicle_state);

    nox_msgs::GetSceneRequest request_for_scene;
    _vehicle.pose.To(request_for_scene.location.pose);
    request_for_scene.location.header = vehicle_state.header;

    // TODO: 超时处理
    if(not _scene_server.SendRequest(request_for_scene))
    {
        Logger::E("Planner") << "Fail to update scene. (sec: " << timer.Watch().Get<Second>() << ")";
        return;
    }

    _scene.From(_scene_server.GetResponse().scene);
    _scene.Refresh({"scene"});
    //endregion

    //region 进行规划
    Trajectory planning_trajectory;
    if(mailboxes.trajectory.SendHistoryCount() != 0)
        planning_trajectory.From(mailboxes.trajectory.LastSend());

    auto result = Process(planning_trajectory);
    //endregion

    //region 处理规划结果
    if(result.Fail())
    {
        Logger::W("Planner") << result.Message();
    }
    else
    {
        trajectory.emplace();
        planning_trajectory.To(trajectory.value());
        planning_trajectory.Refresh({"trajectory"});
    }
    //endregion
}


Result<bool> Planner::Plan(PlannerBase::Frame & frame, type::Trajectory &trajectory, bool enable_stitch)
{
    /// 1. 计算缝合轨迹，或裁短规划轨迹
    if(enable_stitch)
    {
        bool is_replan = false;
        trajectory = _trajectoryStitcher->FromLastTrajectoryByPosition(_vehicle, trajectory, &is_replan);
    }
    else
    {
        auto nearest_index = trajectory.QueryNearestByPosition(_vehicle.pose.t);
        auto nearest_point = trajectory[nearest_index];
        auto forward_index  = trajectory.QueryNearestByTime(nearest_point.t + _param._reserve._forward_time);
        auto backward_index = trajectory.QueryNearestByTime(nearest_point.t - _param._reserve._backward_time);

        trajectory = trajectory.SubTrajectory(backward_index, forward_index);
    }

    frame.stitch = AddressOf(trajectory);

    /// 2. 运用决策器，决策信息放在ReferenceLine中
    for(auto & decider : _deciders)
    {
        decider->Execute(frame);
    }

    /// 3. 运用规划器
    auto planning_result = _algorithm->Plan(frame, trajectory);

    /// 4. 返回处理结果
    if(planning_result.OK())
    {
        return Result(true);
    }
    else
    {
        return Result(false, "Unable to plan because of " + planning_result.Message());
    }
}

Result<bool> Planner::Process(type::Trajectory &last_trajectory)
{
    //region 初始化
    Result<bool> result;
    PlannerBase::Frame frame;
    frame.scene = AddressOf(_scene);
    frame.vehicle = AddressOf(_vehicle);

    for(auto & i : _scene.GuideLines)
    {
        frame.references.push_back(New<ReferenceLine>(*i.second));
    }
    //endregion

    /// 1. 当上一条轨迹为空时，直接重规划
    if (last_trajectory.Empty())
    {
        result = Plan(frame, last_trajectory);
    }
    else
    {
        /// 2. 先检验上一条轨迹的合理性
        auto nearest_index = last_trajectory.QueryNearestByPosition(_vehicle.pose.t);
        auto nearest_point = last_trajectory[nearest_index];

        /// 3. 若车已经远离上一条轨迹，则重规划
        if (_vehicle.pose.t.DistanceTo(nearest_point.pose.t) > _param._threshold._replan_distance)
        {
            result = Plan(frame, last_trajectory);
        }
        else
        {
            /// 4. 若车没有远离轨迹，优先考虑续用该条轨迹，判断上一条轨迹终点与当前位置的关系
            auto last_point = last_trajectory.Back();

            /// 5. 若当前位置离终点太近，则直接重规划
            if(last_point.t - nearest_point.t < _param._threshold._replan_time or last_point.s - nearest_point.s < _param._threshold._replan_distance)
            {
                result = Plan(frame, last_trajectory);
            }
            else if(auto check_result = _algorithm->Check(last_trajectory, frame); check_result.Fail())
            {
                /// 6. 否则检查是否该轨迹是否会发生碰撞，会的话也直接重规划
                Logger::W("Planner") << "Check last trajectory failed. Because of " << check_result.Message();
                result = Plan(frame, last_trajectory);
            }
            else if(last_point.t - nearest_point.t < _param._threshold._extend_time)
            {
                /// 7. 若当前没啥问题，则判断剩余时间，到一定阈值则延长规划。
                if(result = Plan(frame, last_trajectory, false); result.Fail())
                {
                    /// 8. 如果延长规划失败，则全部重规划
                    Logger::W("Planner") << "Extend last trajectory failed";
                    result = Plan(frame, last_trajectory);
                }
            }
        }
    }

    return result;
}





