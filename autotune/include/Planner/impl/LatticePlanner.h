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
            double planning_distance;
            double planning_temporal_length;
            double time_resolution;
            double lane_default_width = 4.0;
        } param;

    private:
        void ComputeFrentState(
            const type::TrajectoryPoint &init_point,
            const type::PathPoint &matched_point,
            OUT math::Derivative<3> &s,
            OUT math::Derivative<3> &l);
    };
}