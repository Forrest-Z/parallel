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
        Result Plan(const type::Trajectory & stitch_trajectory, Frame frame, type::Trajectory & result) override;

        Result PlanOnReferenceLine(
            const type::TrajectoryPoint &init_point,
            Ptr<ReferenceLine> referenceLine,
            Frame frame,
            nox::type::Trajectory & result,
            double & cost);

        Result Check(const type::Trajectory &trajectory, const Frame &frame) override;

    public:
        struct
        {
            double planning_distance = 200;
            double planning_temporal_length = 8;
            double time_resolution = 0.1;
            double lane_default_width = 3.5;
            double cost_not_optimal_reference_line = 2.0;
        } param;

    private:
        void ComputeFrentState(
            const type::TrajectoryPoint &init_point,
            const type::PathPoint &matched_point,
            OUT math::Derivative<2> &s,
            OUT math::Derivative<2> &l);
    };
}