#include <Planner/Planner.h>
#include <Planner/impl/LatticePlanner.h>
#include <Planner/PlannerConfig.h>
#include <Planner/.PlannerModule.h>
#include <Planner/rule/StopLine.h>
#include <Planner/rule/Passable.h>
#include <Planner/rule/PullOver.h>
#include <nox>
#include <iostream>
#include "../../../.param/template/Parameter.h"
#include <any>

using namespace nox::app;
USING_NAMESPACE_NOX;
using std::cout;
using std::endl;

void Planner::Initialize()
{
    _trajectoryStitcher = New<TrajectoryStitcher>();
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
        ->AddRule<rule::Passable>()
        ->AddRule<rule::PullOver>();

    _deciders.push_back(guideDecider);
}

void Planner::InitializeParameters()
{
    _vehicle.param.length.x = params.Vehicle.Physical.Length;
    _vehicle.param.length.y = params.Vehicle.Physical.Width;
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
    Trajectory last_trajectory, new_trajectory;
    if(mailboxes.trajectory.SendHistoryCount() != 0)
        last_trajectory.From(mailboxes.trajectory.LastSend());

    auto result = Process(last_trajectory, new_trajectory);
    //endregion

    //region 处理规划结果
    if(result.Fail())
    {
        Logger::W("Planner") << result.Message();
    }

    trajectory.emplace();
    new_trajectory.To(trajectory.value());
    new_trajectory.Refresh({"trajectory"});
    //endregion
}


Result<bool> Planner::Plan(PlannerBase::Frame & frame, type::Trajectory &new_trajectory, bool should_replan)
{
    /// 1. 计算缝合轨迹，或裁短规划轨迹
    auto & last_trajectory = *frame.stitch;
    if(should_replan)
    {
        last_trajectory = _trajectoryStitcher->InitialTrajectory(_vehicle);
    }
    else
    {
        auto nearest_index = last_trajectory.QueryNearestByPosition(_vehicle.pose.t);
        auto nearest_point = last_trajectory[nearest_index];
        auto forward_index  = last_trajectory.QueryNearestByTime(nearest_point.t + _param._reserve._forward_time);
        auto backward_index = last_trajectory.QueryNearestByTime(nearest_point.t - _param._reserve._backward_time);

        last_trajectory = last_trajectory.SubTrajectory(backward_index, forward_index);
    }

    /// 2. 运用决策器，决策信息放在ReferenceLine中
    for(auto & decider : _deciders)
    {
        decider->Execute(frame);
    }

    /// 3. 运用规划器
    auto planning_result = _algorithm->Plan(frame, new_trajectory);

    /// 4. 返回处理结果
    if(planning_result.OK())
    {
        return Result(true);
    }
    else
    {
        _braking.Plan(frame, new_trajectory);
        return Result(false, "Unable to plan because of " + planning_result.Message() + "Braking ...");
    }
}

Result<bool> Planner::Process(type::Trajectory & last_trajectory, type::Trajectory & new_trajectory)
{
    Result<bool> result;
    PlannerBase::Frame frame; // FIXME: 没必要每一帧都全新构造
    frame.scene = AddressOf(_scene);
    frame.vehicle = AddressOf(_vehicle);
    frame.stitch = AddressOf(last_trajectory);

    for(auto & i : _scene.GuideLines)
    {
        frame.references.push_back(New<ReferenceLine>(*i.second));
    }

    if(auto r = CouldExtend(frame); r.Fail())
    {
        Logger::W("Planner") << "Trajectory has to re-plan because of: " << r.Message();
        result = Plan(frame, new_trajectory, true);
    }
    else if(r.Message() != "no need")
    {
        if(r = Plan(frame, new_trajectory, false); r.Fail())
        {
            // 如果延长规划失败，则全部重规划
            Logger::W("Planner") << "Extend last trajectory failed because of: " << r.Message();
            result = Plan(frame, new_trajectory);
        }
    }
    else
        new_trajectory = last_trajectory;

    return result;
}

Result<bool> Planner::CouldExtend(const PlannerBase::Frame &frame)
{
    auto & last_trajectory = *frame.stitch;
    auto & vehicle = *frame.vehicle;

    if(last_trajectory.Empty())
        return Result(false, "Last trajectory is empty.");

    if(auto r = _trajectoryStitcher->Check(last_trajectory, vehicle); r.Fail())
        return r;

    auto nearest_index = last_trajectory.QueryNearestByPosition(vehicle.pose.t);
    auto nearest_point = last_trajectory[nearest_index];
    auto last_point = last_trajectory.Back();

    if( last_point.t - nearest_point.t < _param._threshold._replan_time or
        last_point.s - nearest_point.s < _param._threshold._replan_distance)
    {
        return Result(false, "Vehicle is nearly reach the end of the trajectory.");
    }

    if(auto r = _algorithm->Check(frame); r.Fail())
        return r;

    if(last_point.t - nearest_point.t < _param._threshold._extend_time)
        return Result(true);
    else
        return Result(true, "no need");
}





