/**
 * @file PiecewiseBrakingTrajectoryGenerator.h
 */

#pragma once

#include <nox>

namespace nox::app
{
    class PiecewiseBrakingTrajectoryGenerator
    {
    public:
        PiecewiseBrakingTrajectoryGenerator() = delete;

        static Ptr<math::Parametric<1>> Generate(
            const double s_target, const double s_curr,
            const double v_target, const double v_curr,
            const double a_comfort, const double d_comfort,
            const double max_time);

        static double ComputeStopDistance(const double v,
                                          const double dec);

        static double ComputeStopDeceleration(const double dist,
                                              const double v);
    };
}