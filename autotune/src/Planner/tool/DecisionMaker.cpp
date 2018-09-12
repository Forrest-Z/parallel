#include <Planner/tool/DecisionMaker.h>

using namespace nox::app;


void DecisionMaker::Execute(vector<ReferenceLine> &referenceLines) const
{
    for(auto & i : referenceLines)
        Execute(i);
}

void DecisionMaker::Execute(ReferenceLine &referenceLine) const
{
    for(const auto & i : _rules)
        i->Apply(i->_decider, referenceLine);
}

DecisionMaker *DecisionMaker::AddRule(DecisionMaker::PtrRule rule)
{
    rule->_decider = this;
    _rules.push_back(rule);
    return this;
}
