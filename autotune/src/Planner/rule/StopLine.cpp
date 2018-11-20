#include <Planner/rule/StopLine.h>

namespace nox::app::rule
{
    void StopLine::Apply(const PlannerBase::Frame & frame, ReferenceLine & referenceLine) const
    {
        if(referenceLine.stopLine)
        {
            auto frenet = referenceLine.CalculateFrenet(frame.stitch->Back().pose);
            double ds = referenceLine.stopLine->s - frenet.s;

            if(ds >= 0 and ds <= 50)
                referenceLine.AddCost(ReferenceLine::Important, 1);
        }
    }
}