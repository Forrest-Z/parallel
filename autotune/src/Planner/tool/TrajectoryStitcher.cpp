#include <Planner/tool/TrajectoryStitcher.h>

using namespace nox::app;
USING_NAMESPACE_NOX;

Trajectory TrajectoryStitcher::InitialTrajectory(const Vehicle &vehicle)
{
    return ComputeTrajectory(vehicle.pose.t, vehicle.pose.theta, vehicle.kappa, vehicle.v.x, vehicle.a.x, param.initial_stitch_time);
}

Trajectory TrajectoryStitcher::ComputeTrajectory(Position position, double theta, double kappa, double v, double a, double time_sum)
{
    double distance_sum = v * time_sum;

    TrajectoryPoint point;
    point.pose.x = position.x;
    point.pose.y = position.y;
    point.pose.theta = theta;
    point.v = v;
    point.a = a;
    point.kappa = kappa;

    type::Trajectory result;
    std::vector<double> vs;
    for(double s : range(0, param.density, distance_sum))
        vs.push_back(s);
    if(vs.empty() or vs.back() != distance_sum)
        vs.push_back(distance_sum);

    for(double s : vs)
    {
        result.Add(point.MoveByDistance(s));
    }

    return result;
}

Trajectory TrajectoryStitcher::FromLastTrajectoryByPosition(
    const Trajectory &last_trajectory,
    const Vehicle &vehicle,
    Result<bool> * status)
{
    if(auto r = Check(last_trajectory, vehicle); r.Fail())
    {
        if(status)
            *status = r;
        Logger::W("TrajectoryStitcher") << "Has to Re-plan because of " << r.Message();
        return InitialTrajectory(vehicle);
    }

    size_t nearest_index    = last_trajectory.QueryNearestByPosition(vehicle.pose.t);
    auto & nearest_point    = last_trajectory[nearest_index];
    size_t forward_index    = last_trajectory.QueryNearestByTime(nearest_point.t + param.stitch_time);
    size_t backward_index   = last_trajectory.QueryNearestByTime(nearest_point.t - param.stitch_time);
    type::Trajectory result = last_trajectory.SubTrajectory(backward_index, forward_index);

    if(status)
        *status = Result(true);
    return result;
}

Result<bool> TrajectoryStitcher::Check(const Trajectory &trajectory, const Vehicle &vehicle) const
{
    if(trajectory.Length() < param.threshold.replan.too_short)
        return Result(false, "Trajectory is too short.");

    size_t nearest_index = trajectory.QueryNearestByPosition(vehicle.pose.t);
    auto & nearest_point = trajectory[nearest_index];

    double offset = nearest_point.pose.t.DistanceTo(vehicle.pose.t);
    if(offset > param.threshold.replan.distance)
        return Result(false, "Vehicle is too far from the trajectory.");

    double speed_diff = abs(vehicle.v.x - nearest_point.v);
    if(speed_diff > param.threshold.replan.speed_diff)
        return Result(false, "Vehicle's speed is too different from the matched point on the trajectory.");

    return Result(true);
}
