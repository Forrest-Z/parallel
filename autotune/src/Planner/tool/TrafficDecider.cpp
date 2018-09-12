#include <Planner/tool/TrafficDecider.h>

using namespace nox::app;


void TrafficDecider::Rule::Apply(DecisionMaker *decider, ReferenceLine &referenceLine) const
{
    auto * impl = dynamic_cast<TrafficDecider *>(decider);
    if(impl)
        Apply(impl, referenceLine);
}

TrafficDecider::TrafficDecider(TrafficDecider::Ptr<nox::type::Scene> &scene, TrafficDecider::Ptr<nox::type::Vehicle> &vehicle)
    : scene(scene), vehicle(vehicle)
{}
