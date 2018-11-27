/**
 * @brief lattice轨迹生成器
 */
#pragma once

#include <nox>
#include <Planner/type/STGraph.h>
#include "LatticeCommon.h"
#include "LatticeSampler.h"
#include <Planner/type/ReferenceLine.h>
#include <Planner/tool/PredictionQuerier.h>

namespace nox::app
{
    class LatticeGenerator
    {
    public:
        LatticeGenerator(
            const math::Derivative<2> & s,
            const math::Derivative<2> & l,
            Ptr<STGraph> path_time_graph,
            Ptr<PredictionQuerier> prediction_querier
        );

        void GenerateBundles(
            Ptr<ReferenceLine> reference,
            lattice::Bundle & lon,
            lattice::Bundle & lat
        ) const;

        void Combine(
            Ptr<ReferenceLine> reference,
            const math::Parametric<1> & lon,
            const math::Parametric<1> & lat,
            type::Trajectory & result
        ) const;

    private:
        using States = std::vector<lattice::State>;

        void GenerateLongitudinalBundle(
            Ptr<ReferenceLine> reference,
            lattice::Bundle  & result
        ) const;

        void GenerateLateralBundle(
            lattice::Bundle  & result
        ) const;

        void GenerateSpeedProfilesForCruising(
            double target_speed,
            lattice::Bundle & lon
        ) const;

        void GenerateSpeedProfilesForStopping(
            double distance,
            lattice::Bundle & lon
        ) const;

        void GenerateSpeedProfilesForObstacles(
            lattice::Bundle & lon
        ) const;

        void GenerateQuarticBundle(
            const lattice::State & init_state,
            Ptr<States> end_states,
            lattice::Bundle & result
        ) const;

        void GenerateQuinticBundle(
            const lattice::State & init_state,
            Ptr<States> end_states,
            lattice::Bundle & result
        ) const;

    private:
        lattice::State _init_lon_state;
        lattice::State _init_lat_state;
        LatticeSampler _sampler;

        struct
        {
            double _time_resolution = 0.1;
            double _planning_temporal_length = 8.0;
            double _delta_s_lateral_optimization = 2.0;
            double _default_cruising_speed = 5.0;
            double _vehicle_length = 4.0;
        } _param;
    };
}