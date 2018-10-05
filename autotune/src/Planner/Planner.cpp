#include <Planner/Planner.h>
#include <Planner/rule/Junction.h>
#include <Planner/rule/Overtake.h>
#include <Planner/rule/SignalLight.h>
using namespace nox::app;
USING_NAMESPACE_NOX;


void Planner::InitializeDeciders()
{
    auto traffic_decider = std::make_shared<TrafficDecider>(AddressOf(_scene), AddressOf(_vehicle));
    traffic_decider->AddRule<rule::Junction>()
                   ->AddRule<rule::SignalLight>()
                   ->AddRule<rule::Overtake>();

    _deciders.push_back(traffic_decider);
}

Planner::Result Planner::Process(Ptr<type::Vehicle> vehicle, Ptr<type::Scene> scene, type::Trajectory &trajectory)
{
    /// 1. 计算缝合轨迹
    bool is_replan = false;
    trajectory = _trajectoryStitcher->FromLastTrajectory(*vehicle, GetPeriod(), trajectory, is_replan);

    /// 2. 封装车道线为reference_line
    vector<ReferenceLine> references;
    for(auto & i : scene->lanes)
    {
        references.emplace_back(*i.second);
    }

    /// 3. 运用决策器，决策信息放在ReferenceLine中
    for(auto & decider : _deciders)
    {
        decider->Execute(references);
    }

    /// 4. 构造Frame
    PlannerBase::Frame frame{AddressOf(references), scene, vehicle};

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
