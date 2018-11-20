#include <Planner/tool/DecisionMaker.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


void DecisionMaker::Execute(const PlannerBase::Frame & frame) const
{
    for(auto & i : frame.references)
        Execute(frame, *i);
}

void DecisionMaker::Execute(const PlannerBase::Frame & frame, ReferenceLine & referenceLine) const
{
    for(const auto & i : _rules)
        i->Apply(frame, referenceLine);
}

Ptr<DecisionMaker> DecisionMaker::AddRule(Ptr<DecisionMaker::Rule> rule)
{
    _rules.push_back(rule);
    return AddressOf(*this);
}
