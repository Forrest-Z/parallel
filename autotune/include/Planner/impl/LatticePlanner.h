/**
 * @brief Lattice Planner Impl
 */
#pragma once

#include <Planner/impl/PlannerBase.h>

namespace nox::app
{
    class LatticePlanner : public PlannerBase
    {
    public:
        Result PlanOnReferenceLine(
            const type::TrajectoryPoint &init_point,
            ReferenceLine &referenceLine,
            Frame frame,
            Ptr<nox::type::Trajectory> & result) final;

    public:
        struct
        {
            double planning_distance = 50;
            double planning_temporal_length = 8;
            double time_resolution = 0.1;
            double lane_default_width = 3.5;
        } param;

    private:
        void ComputeFrentState(
            const type::TrajectoryPoint &init_point,
            const type::PathPoint &matched_point,
            OUT math::Derivative<2> &s,
            OUT math::Derivative<2> &l);
    };
}