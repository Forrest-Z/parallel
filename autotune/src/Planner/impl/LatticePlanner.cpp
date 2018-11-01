#include <Planner/impl/LatticePlanner.h>
#include <Planner/type/STGraph.h>
#include <Planner/tool/PredictionQuerier.h>
#include <Planner/tool/lattice/LatticeGenerator.h>
#include <Planner/tool/lattice/LatticeEvaluator.h>
#include <Planner/tool/CollisionChecker.h>
USING_NAMESPACE_NOX;
using namespace nox::app;
using namespace nox::type;
using std::endl;

PlannerBase::Result LatticePlanner::PlanOnReferenceLine(
    const type::TrajectoryPoint &init_point,
    Ptr<ReferenceLine> reference,
    PlannerBase::Frame frame,
    type::Trajectory & result)
{
    analyze_init(LatticePlanner);
    analyze_enable(false);

    analyze(1. 匹配起点在轨迹上的最近点);
    size_t matched_index = reference->path.QueryNearestByPosition(init_point.pose.t);
    const auto matched_point = reference->path.at(matched_index);

    analyze(2. 计算起点与最近点的Frenet坐标);
    math::Derivative<2> s, l;
    ComputeFrentState(init_point, matched_point, s, l);

    analyze(3. 计算障碍物时空分布图);
    auto st_graph = New<STGraph>
    (
        frame.scene,                          // 所有障碍物
        reference,                            // 沿着当前参考线建立图
        s[0], s[0] + param.planning_distance, // 图的距离范围
        0,    param.planning_temporal_length, // 图的时间范围
        param.lane_default_width,             // 图的宽度范围
        param.time_resolution                 // 时间分辨率
    );

    analyze(4. 分别生成纵向和横向的候选轨迹);
    auto prediction_querier = New<PredictionQuerier>
    (
        frame.scene,
        reference
    );

    LatticeGenerator generator
    (
        s, l,
        st_graph,
        prediction_querier
    );

    lattice::Bundle lon_bundle;
    lattice::Bundle lat_bundle;
    generator.GenerateBundles
    (
        reference,
        lon_bundle,
        lat_bundle
    );

    analyze(5. 评估轨迹);
    LatticeEvaluator evaluator
    (
        s,                        // 规划起始状态
        lon_bundle,               // 纵向分量
        lat_bundle,               // 横向分量
        frame.vehicle,            // 车体参数与状态
        st_graph,                 // 障碍物时空图
        reference                 // 引导参考线
    );

    analyze(6. 遍历所有候选轨迹，进行碰撞检测与约束检测);
    ConstraintChecker constraint_checker(frame.vehicle);
    CollisionChecker collision_checker(frame.scene, s[0], l[0], reference);

    while (evaluator.HasNext())
    {
        auto candidate = evaluator.Next();

        Trajectory & trajectory = result;
        generator.Combine(reference, *candidate.lon, *candidate.lat, trajectory);

        if(!constraint_checker.CheckTrajectory(trajectory))
            continue;

        if(collision_checker.InCollision(trajectory))
            continue;

        auto lon = std::dynamic_pointer_cast<lattice::Curve>(candidate.lon);
        auto lat = std::dynamic_pointer_cast<lattice::Curve>(candidate.lat);
        Logger::D("LatticePlanner")
            << endl
            << "Pick lon(p, v, t) = " << lon->target_position << ", " << lon->target_speed << ", " << lon->target_time << endl
            << "Pick lat(p, v, t) = " << lat->target_position << ", " << lat->target_speed << ", " << lat->target_time << endl
            << "Cost: " << candidate.cost_sum << endl
            << "1. lon objective: " << candidate.costs[0] << endl
            << "2. lon jerk:      " << candidate.costs[1] << endl
            << "3. lon collision: " << candidate.costs[2] << endl
            << "4. centripetal a: " << candidate.costs[3] << endl
            << "5. lat offset:    " << candidate.costs[4] << endl
            << "6. lat comfort:   " << candidate.costs[5] << endl;

        return Result(ErrorCode::Success);
    }

    return Result(ErrorCode::Fail);
}

void LatticePlanner::ComputeFrentState(
    const TrajectoryPoint &init_point,
    const PathPoint &matched_point,
    nox::math::Derivative<2> &s,
    nox::math::Derivative<2> &l)
{
    math::Cartesian2Frenet(
        math::Cartesian(init_point.pose.x, init_point.pose.y, init_point.pose.theta),
        math::Cartesian(matched_point.pose.x, matched_point.pose.y, matched_point.pose.theta),
        init_point.v, init_point.a, init_point.kappa,
        matched_point.s, math::Derivative<1>{matched_point.kappa, matched_point.dkappa},
        OUT s, OUT l
    );
}
