#include <Planner/Planner.h>
#include <Planner/rule/Junction.h>
#include <Planner/rule/Overtake.h>
#include <Planner/rule/SignalLight.h>
using namespace nox::app;
USING_NAMESPACE_NOX;


void Planner::InitializeDeciders()
{
    auto traffic_decider = std::make_shared<TrafficDecider>(_scene, _vehicle);
    traffic_decider->AddRule<rule::Junction>()
                   ->AddRule<rule::SignalLight>()
                   ->AddRule<rule::Overtake>();

    _deciders.push_back(traffic_decider);
}
