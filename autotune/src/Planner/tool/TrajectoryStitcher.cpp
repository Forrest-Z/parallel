#include <Planner/tool/TrajectoryStitcher.h>

using namespace nox::app;
USING_NAMESPACE_NOX;

type::Trajectory TrajectoryStitcher::InitialTrajectory(const type::Vehicle &vehicle)
{
    return ComputeTrajectory(vehicle.position(), vehicle.theta(), vehicle.kappa(), vehicle.vx(), param.initial_stitch_time);
}

type::Trajectory TrajectoryStitcher::FromLastTrajectory(
    const type::Vehicle &vehicle,
    double last_cycle_time,
    const type::Trajectory &last_trajectory,
    OUT bool *replan)
{
    auto MakeResult = [&](type::Trajectory result, bool is_replan)
    {
        if(replan) *replan = is_replan;
        return std::move(result);
    };

    /// 旧轨迹为空，返回初始化轨迹
    if(last_trajectory.Empty())
    {
        return MakeResult(InitialTrajectory(vehicle), true);
    }

    /// 按时间匹配当前位置在旧轨迹上的位置
    size_t matched_index = last_trajectory.QueryNearest(last_cycle_time);
    auto   matched_point = last_trajectory[matched_index];

    /// 时间最近点不在旧轨迹上，则返回初始化轨迹
    if( (matched_index == 0 and matched_point.t < last_trajectory.Front().t)
            or
        (matched_index == last_trajectory.Size() - 1 and matched_point.t > last_trajectory.Back().t) )
    {
        return MakeResult(InitialTrajectory(vehicle), true);
    }

    /// 检查时间最近点的位移是否过大
    size_t nearest_index = last_trajectory.QueryNearest(vehicle.position());
    auto   nearest_point = last_trajectory[nearest_index];
    double lateral_offset      = matched_point.LateralFrom(nearest_point);
    double longitudinal_offset = matched_point.LongitudinalFrom(nearest_point);

    if(lateral_offset > param.threshold.replan.lateral_offset
            or
       longitudinal_offset > param.threshold.replan.longitudinal_offset)
    {
        return MakeResult(InitialTrajectory(vehicle), true);
    }

    /// 取出缝合轨迹（可能旧轨迹不够长，后边会延长）
    size_t forward_index  = last_trajectory.QueryNearest(matched_point.t + param.stitch_time);
    size_t backward_index = last_trajectory.QueryNearest(matched_point.t - param.stitch_time);
    type::Trajectory result = last_trajectory.SubTrajectory(backward_index, forward_index);

    const auto forward_point = last_trajectory[forward_index];
    double rest_forward_time = matched_point.t + param.stitch_time - forward_point.t;
    if(!Real::IsZero(rest_forward_time))
    {
        type::Trajectory rest_trajectory = ComputeTrajectory(
            forward_point.position(),
            forward_point.theta(),
            vehicle.kappa(),
            forward_point.v,
            rest_forward_time);
        result += rest_trajectory;
    }

    return result;
}

type::Trajectory TrajectoryStitcher::ComputeTrajectory(type::Position position, double theta, double kappa, double v, double time_sum)
{
    double distance = 0;
    double distance_sum = v * time_sum;
    double time = 0;
    double time_step = 99999;
    if(param.density != 0 && distance_sum != 0)
        time_step = time_sum / (distance_sum / param.density);

    TrajectoryPoint point;
    point.theta() = theta;
    point.v = v;
    point.a = 0;
    point.kappa() = kappa;

    type::Trajectory result;
    while (distance <= distance_sum) // 最后一个点可能被抛弃，现在暂时不管
    {
        point.x() = position.x;
        point.y() = position.y;
        point.z() = position.z;
        point.s() = distance;
        point.t = time;

        result.Add(point);
        time += time_step;
        distance += param.density;
        position.Move(param.density);
    }

    return result;
}