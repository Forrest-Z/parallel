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
        auto tp = point.MoveByDistance(s);
        if(point.v == 0)
        {
            if(result.Empty())
                tp.t = 0;
            else
                tp.t = result.Back().t;
        }
        result.Add(tp);
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

    if(status)
        *status = Result(true);

    return StitchTrajectoryByPosition(last_trajectory, vehicle, param.stitch_time);
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

    double angle_diff = abs(Radian(vehicle.pose.theta - nearest_point.pose.theta).Warp(Angle::Degree));
    if(angle_diff > 80.0)
        return Result(false, "Vehicle's heading is too different from the matched point on the trajectory.");


    return Result(true);
}

Trajectory TrajectoryStitcher::StitchTrajectoryByPosition(
    const Trajectory &last_trajectory,
    const Vehicle &vehicle,
    double stitch_time)
{
    if(last_trajectory.Empty())
        return InitialTrajectory(vehicle);

    size_t nearest_index    = last_trajectory.QueryNearestByPosition(vehicle.pose.t);
    auto & nearest_point    = last_trajectory[nearest_index];
    size_t forward_index    = last_trajectory.QueryNearestByTime(nearest_point.t + stitch_time);
    size_t backward_index   = last_trajectory.QueryNearestByTime(nearest_point.t - stitch_time);
    type::Trajectory result = last_trajectory.SubTrajectory(backward_index, forward_index);

    std::cout << "[backward, forward]: " << backward_index << ", " << forward_index << std::endl;
    std::cout << "sub length: " << result.Length() << std::endl;

    return result;
}
