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

    auto result = Process(AddressOf(_vehicle), AddressOf(_scene), planning_trajectory);

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
    }

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
}


Planner::Result Planner::Process(Ptr<type::Vehicle> vehicle, Ptr<type::Scene> scene, type::Trajectory &trajectory)
{
    /// 1. 计算缝合轨迹
    bool is_replan = false;
//     trajectory = _trajectoryStitcher->FromLastTrajectoryByPosition(*vehicle, trajectory, &is_replan);
    trajectory = _trajectoryStitcher->InitialTrajectory(*vehicle);

    /// 2. 封装车道线为reference_line
    PlannerBase::Frame frame;
    frame.scene = scene;
    frame.vehicle = vehicle;

    for(auto & i : scene->GuideLines)
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



