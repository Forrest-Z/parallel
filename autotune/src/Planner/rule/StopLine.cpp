#include <Planner/rule/StopLine.h>

namespace nox::app::rule
{
    void StopLine::Apply(Ptr<GuideDecider> decider, Ptr<ReferenceLine> referenceLine) const
    {
        if(referenceLine->stopLine)
        {
            auto frenet = referenceLine->CalculateFrenet(decider->vehicle);
            double ds = referenceLine->stopLine->s - frenet.s;

            if(ds >= 0 and ds <= 50)
                referenceLine->AddCost(ReferenceLine::Important, 1);
        }
    }
}