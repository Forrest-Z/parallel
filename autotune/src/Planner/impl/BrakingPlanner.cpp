#include <Planner/impl/BrakingPlanner.h>
#include <Planner/tool/PiecewiseBrakingTrajectoryGenerator.h>
#include <nox>


namespace nox::app
{

    Result<bool> BrakingPlanner::Plan(const PlannerBase::Frame &frame, type::Trajectory &result)
    {
        //FIXME 目前只能尽可能快地刹车，之后需要不用太快刹车，且检查这条轨迹的合法性
        double dec_a = frame.vehicle->param.limit.lon.a.Lower;
        auto dec_traj = PiecewiseBrakingTrajectoryGenerator::Generate(
            0, frame.vehicle->v.x, 0, dec_a
        );

        double dt = 0.1;
        double T = dec_traj->Upper();
        std::vector<double> ts;
        for(double t : range(dt, dt, T))
            ts.push_back(t);

        if(ts.empty() or ts.back() != T)
            ts.push_back(T);

        TrajectoryPoint init_point;
        init_point.pose = frame.vehicle->pose;
        init_point.v    = frame.vehicle->v.x;
        init_point.a    = frame.vehicle->a.x;

        for(double t : ts)
        {
            double s = dec_traj->Calculate(0, t);

            auto p = init_point.MoveByDistance(s);
            p.t = init_point.t + t;
            p.v = dec_traj->Calculate(1, t);
            p.a = dec_traj->Calculate(2, t);
            result.Add(p);
        }

        return Result(true);
    }

    Result<bool> BrakingPlanner::Check(const PlannerBase::Frame &frame)
    {
        // Do Nothing here ...
        return Result(true);
    }
}

