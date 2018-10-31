#include <utility>

#include <Planner/tool/GuideDecider.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


void GuideDecider::Rule::Apply(Ptr<DecisionMaker> decider, Ptr<ReferenceLine> referenceLine) const
{
    Apply(std::dynamic_pointer_cast<GuideDecider>(decider), referenceLine);
}

GuideDecider::GuideDecider(Ptr<Vehicle> vehicle)
    : vehicle(std::move(vehicle))
{}
