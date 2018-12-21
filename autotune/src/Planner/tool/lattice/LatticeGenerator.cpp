#include <utility>
#include <Planner/PlannerConfig.h>
#include <utility>
#include <Planner/type/QuadraticAccelerationCurve.h>
#include <Planner/tool/lattice/LatticeGenerator.h>
#include <nox>

USING_NAMESPACE_NOX;
using namespace nox::app;


LatticeGenerator::LatticeGenerator(
    const nox::math::Derivative<2> &s,
    const nox::math::Derivative<2> &l,
    Ptr<STGraph> path_time_graph,
    Ptr<PredictionQuerier> prediction_querier
)
    : _init_lon_state(s, 0), _init_lat_state(l, 0),
      _sampler(s, l, std::move(path_time_graph), std::move(prediction_querier))
{
    auto & vehicle = cache::ReadEgoVehicle();
    _param._vehicle_length = vehicle.param.length.x;
}

void LatticeGenerator::GenerateBundles(Ptr<ReferenceLine> reference, lattice::Bundle &lon, lattice::Bundle &lat) const
{
    GenerateLongitudinalBundle(std::move(reference), lon);
    GenerateLateralBundle(lat);

    Logger::D("LatticeGenerator") << "Lon Bundles: " << lon.size() << " ; Lat Bundles: " << lat.size();
}

void LatticeGenerator::GenerateLongitudinalBundle(Ptr<ReferenceLine> reference, lattice::Bundle &result) const
{
    double s = _init_lon_state[0];
    GenerateSpeedProfilesForCruising(reference->CruisingSpeed(s), result);
    GenerateSpeedProfilesForObstacles(result);

    double next_stop_line = reference->GetNextStopLine(s);
    if(next_stop_line <= reference->Length())
    {
        Logger::D("LatticeGenerator") << "Stop Line: " << next_stop_line;
        GenerateSpeedProfilesForStopping(next_stop_line - _param._vehicle_length, result);
    }
}

void LatticeGenerator::GenerateLateralBundle(lattice::Bundle &result) const
{
    auto end_states = _sampler.SampleLatStates();
    GenerateQuinticBundle(_init_lat_state, end_states, result);

    double s_min = 0;
    double s_max = _param._delta_s_lateral_optimization;

}

void LatticeGenerator::GenerateSpeedProfilesForCruising(double target_speed, lattice::Bundle &lon) const
{
    auto end_states = _sampler.SampleLonStatesForCruising(target_speed);
    if(end_states->empty())
    {
        return;
    }

    GenerateQuarticBundle(_init_lon_state, end_states, lon);
}

void LatticeGenerator::GenerateSpeedProfilesForStopping(double distance, lattice::Bundle &lon) const
{
    auto end_states = _sampler.SampleLonStatesForStopping(distance);
    if(end_states->empty())
    {
        Logger::D("LatticeGenerator") << "There is a stop line but not samples for stopping.";
        return;
    }

    GenerateQuadraticAccelerationBundle(_init_lon_state, end_states, lon);
//    GenerateQuinticBundle(_init_lon_state, end_states, lon);
}

void LatticeGenerator::GenerateSpeedProfilesForObstacles(lattice::Bundle &lon) const
{
    auto end_states = _sampler.SampleLonStatesForObstacles();
    if(end_states->empty())
    {
        return;
    }

    GenerateQuinticBundle(_init_lon_state, end_states, lon);
}

void LatticeGenerator::GenerateQuarticBundle(
    const lattice::State &init_state,
    Ptr<LatticeGenerator::States> end_states,
    lattice::Bundle &result) const
{
    result.reserve(end_states->size() + result.size());
    for(auto & end_state : *end_states)
    {
        auto lattice_curve = std::make_shared<lattice::Curve>(
            Ptr<math::Parametric<1>>(new math::QuarticCurve(
                init_state,
                end_state,
                end_state.t
            ))
        );

        lattice_curve->state = end_state;
        result.push_back(lattice_curve);
    }
}

void LatticeGenerator::GenerateQuinticBundle(
    const lattice::State &init_state,
    Ptr<LatticeGenerator::States> end_states,
    lattice::Bundle &result) const
{
    result.reserve(end_states->size() + result.size());
    for(auto & end_state : *end_states)
    {
        auto lattice_curve = New<lattice::Curve>(
            Ptr<math::Parametric<1>>(new math::QuinticCurve(
                init_state,
                end_state,
                end_state.t
            ))
        );

        lattice_curve->state = end_state;
        result.push_back(lattice_curve);
    }
}

void LatticeGenerator::GenerateQuadraticAccelerationBundle(
    const lattice::State &init_state,
    Ptr<States> end_states,
    lattice::Bundle &result) const
{
    result.reserve(end_states->size() + result.size());
    for(auto & end_state : *end_states)
    {
        auto lattice_curve = New<lattice::Curve>(
            Ptr<math::Parametric<1>>(new QuadraticAccelerationCurve(
                init_state,
                end_state,
                end_state.t
            ))
        );

        lattice_curve->state = end_state;
        result.push_back(lattice_curve);
    }
}





