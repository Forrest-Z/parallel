#include <Planner/rule/Passable.h>

namespace nox::app::rule
{
    void Passable::Apply(const PlannerBase::Frame & frame, ReferenceLine & referenceLine) const
    {
        if(not referenceLine.passable)
        {
            auto & planning_start_point = frame.stitch->Back();
            auto frenet = referenceLine.CalculateFrenet(planning_start_point.pose);
            double ds = referenceLine.Length() - frenet.s - planning_start_point.v * 8;

            if(ds < 80.0)
            {
                referenceLine.AddCost(ReferenceLine::Definite, 1);
                referenceLine.Kill();
            }
            else
                referenceLine.AddCost(ReferenceLine::Available, 1);
        }
    }
}

