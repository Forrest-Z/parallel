/**
 * @file LatticeCombiner.h，将两个lattice合成Trajectory
 */

#pragma once

#include <Planner/type/ReferenceLine.h>

namespace nox::app
{
    class LatticeCombiner
    {
    public:
        static type::Trajectory Combine(
            const ReferenceLine & reference,
            const math::Parametric<1> & lon,
            const math::Parametric<1> & lat,
            double time_resolution,
            double temporal_length
        );
    };
}