#include <Planner/rule/StopLine.h>

namespace nox::app::rule
{
    void StopLine::Apply(const PlannerBase::Frame & frame, ReferenceLine & referenceLine) const
    {
        double s0 = frame.stitch->Back().s;
        double stop_line = referenceLine.GetNextStopLine(s0);

        if(stop_line <= referenceLine.Length())
        {
            auto frenet = referenceLine.CalculateFrenet(frame.stitch->Back().pose);
            double ds = stop_line - frenet.s;

            if(ds >= 0 and ds <= 50)
                referenceLine.AddCost(ReferenceLine::Important, 1);
        }
    }
}