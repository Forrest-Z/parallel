#include <Planner/tool/ConstraintChecker.h>
#include <nox>

using namespace nox::app;


ConstraintChecker::ConstraintChecker(nox::Ptr<nox::type::Vehicle> vehicle)
    : _vehicle(vehicle)
{

}

bool ConstraintChecker::CheckLongitudinal(nox::Ptr<nox::math::Parametric<1>> lon_traj) const
{
    for(double t : range(0, _param._time_resolution, lon_traj->Boundary()))
    {
        double v = lon_traj->Calculate(1, t);
        if(v > _vehicle->param.limit.lon.v)
            return false;

        double a = lon_traj->Calculate(2, t);
        if(a > _vehicle->param.limit.lon.a)
            return false;

        double j = lon_traj->Calculate(3, t);
        if(j > _vehicle->param.limit.lon.jerk)
            return false;
    }

    return true;
}

bool ConstraintChecker::CheckTrajectory(const nox::type::Trajectory &trajectory) const
{
    for(auto & i : trajectory)
    {
        if(i.t > _param._planning_temporal_length)
            break;

        if(i.v > _vehicle->param.limit.lon.v)
            return false;

        if(i.a > _vehicle->param.limit.lon.a)
            return false;

        if(i.kappa > _vehicle->param.limit.kappa)
            return false;
    }

    for(int i = 0, end = trajectory.Size() - 1; i < end; ++i)
    {
        auto & p0 = trajectory[i];
        auto & p1 = trajectory[i+1];

        if(p1.t > _param._planning_temporal_length)
            break;

        double dt = p1.t - p0.t;
        double d_lon_a = p1.a - p0.a;
        double lon_jerk = d_lon_a / dt;

        if(lon_jerk > _vehicle->param.limit.lon.jerk)
            return false;

        double lat_a = p1.v * p1.v * p1.kappa;
        if(lat_a > _vehicle->param.limit.lat.a)
            return false;

        double d_lat_a = p1.v * p1.v * p1.kappa - p0.v * p0.v * p0.kappa;
        double lat_jerk = d_lat_a / dt;
        if(lat_jerk > _vehicle->param.limit.lat.jerk)
            return false;
    }

    return true;
}
