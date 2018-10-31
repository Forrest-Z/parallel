#include <Planner/tool/DecisionMaker.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


void DecisionMaker::Execute(vector<Ptr<ReferenceLine>> &referenceLines) const
{
    for(auto & i : referenceLines)
        Execute(i);
}

void DecisionMaker::Execute(Ptr<ReferenceLine> referenceLine) const
{
    assert(referenceLine);
    for(const auto & i : _rules)
        i->Apply(i->_decider, referenceLine);
}

Ptr<DecisionMaker> DecisionMaker::AddRule(Ptr<DecisionMaker::Rule> rule)
{
    rule->_decider = AddressOf(*this);
    _rules.push_back(rule);
    return AddressOf(*this);
}
