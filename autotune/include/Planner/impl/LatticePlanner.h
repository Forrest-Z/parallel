/**
 * @brief Lattice Planner Impl
 */
#pragma once

#include <Planner/PlannerBase.h>

namespace nox::app
{
    class LatticePlanner : public PlannerBase
    {
    public:
        Result<bool> Plan(const Frame & frame, type::Trajectory & result) override;

        Result<bool> PlanOnReferenceLine(
            const Frame                 & frame,
            const type::TrajectoryPoint & init_point,
            Ptr<ReferenceLine>            referenceLine,
            nox::type::Trajectory       & result,
            double                      & cost);

        Result<bool> Check(const Frame &frame) override;

    public:
        struct
        {
            double planning_distance = 200;
            double planning_temporal_length = 8;
            double time_resolution = 0.1;
            double lane_default_width = 3.5;
            double cost_not_optimal_reference_line = 1.0;
        } param;

    private:
        void ComputeFrentState(
            const type::TrajectoryPoint &init_point,
            const type::PathPoint &matched_point,
            OUT math::Derivative<2> &s,
            OUT math::Derivative<2> &l);

        /**
         * 处理轨迹防止启动时速度为零（若轨迹较短，且为停车轨迹，则不处理）
         * @param trajectory
         */
        void LaunchTrajectory(type::Trajectory & trajectory, Ptr<Vehicle> vehicle);
    };
}