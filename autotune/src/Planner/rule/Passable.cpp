#include <Planner/rule/Passable.h>

namespace nox::app::rule
{
    void Passable::Apply(Ptr<GuideDecider> decider, Ptr<ReferenceLine> referenceLine) const
    {
        if(not referenceLine->passable)
        {
            auto frenet = referenceLine->CalculateFrenet(decider->vehicle);
            double ds = referenceLine->Length() - frenet.s;

            if(ds < 50.0)
                referenceLine->AddCost(ReferenceLine::Definite, 1);
            else
                referenceLine->AddCost(ReferenceLine::Available, 1);
        }
    }
}

