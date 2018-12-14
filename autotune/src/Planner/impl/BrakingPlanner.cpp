#include <Planner/impl/BrakingPlanner.h>
#include <Planner/tool/PiecewiseBrakingTrajectoryGenerator.h>
#include <nox>

namespace nox::app
{

    Result<bool> BrakingPlanner::Plan(const PlannerBase::Frame &frame, type::Trajectory &result)
    {
        //FIXME 目前只能尽可能快地刹车，之后需要不用太快刹车，且检查这条轨迹的合法性
        auto dec_traj = PiecewiseBrakingTrajectoryGenerator::Generate(
            0, frame.vehicle->v.x, 0, frame.vehicle->param.limit.lon.a.Lower
        );

        double dt = 0.1;
        double T = dec_traj->Boundary();
        std::vector<double> ts;
        for(double t : range(dt, dt, T))
            ts.push_back(t);

        if(ts.empty() or ts.back() != T)
            ts.push_back(T);

        auto init_point = frame.stitch->Back();
        result = *frame.stitch;

        double last_s = 0;
        for(double t : ts)
        {
            double s = dec_traj->Calculate(0, t);

            if(s - last_s > 1.0 or t == T)
            {
                auto p = init_point.MoveByDistance(s);
                p.t = init_point.t + t;
                p.v = dec_traj->Calculate(1, t);
                p.a = dec_traj->Calculate(2, t);
                result.Add(p);

                last_s = s;
            }
        }

        return Result(true);
    }

    Result<bool> BrakingPlanner::Check(const PlannerBase::Frame &frame)
    {
        // Do Nothing here ...
        return Result(true);
    }
}

