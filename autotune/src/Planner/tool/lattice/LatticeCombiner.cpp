#include <Planner/tool/lattice/LatticeCombiner.h>
USING_NAMESPACE_NOX;

namespace nox::app
{

    type::Trajectory LatticeCombiner::Combine(
        const ReferenceLine &reference,
        const math::Parametric<1> &lon,
        const math::Parametric<1> &lat,
        double time_resolution,
        double temporal_length)
    {
        double s0 = lon.Calculate(0, 0);
        double s_max = reference.path.Back().s;
        double last_s = -Real::Epsilon;
        Trajectory result;

        for(double t : range(0, time_resolution, temporal_length))
        {
            math::Derivative<2> s, l;
            s[0] = lon.Calculate(0, t);

            if (last_s > 0)
                s[0] = std::max(last_s, s[0]);
            last_s = s[0];

            if (s[0] > s_max) break;

            s[1] = std::max(Real::Epsilon, lon.Calculate(1, t));
            s[2] = lon.Calculate(2, t);

            double ds = s[0] - s0;
            l[0] = lat.Calculate(0, ds);
            l[1] = lat.Calculate(1, ds);
            l[2] = lat.Calculate(2, ds);

            auto nearest_point = reference.path.PointAtDistance(s[0]);

            TrajectoryPoint point;
            math::Cartesian original_point;

            math::Frenet2Cartesian(
                math::Cartesian(nearest_point.pose.x, nearest_point.pose.y, nearest_point.pose.theta),
                nearest_point.s, nearest_point.kappa, nearest_point.dkappa,
                s, l,
                original_point,
                point.kappa,
                point.v,
                point.a
            );

            point.pose.x = original_point.x;
            point.pose.y = original_point.y;
            point.pose.theta = original_point.theta;
            point.t = t;

            result.Add(point);
        }

        return result;
    }
}
