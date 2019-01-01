#include <Planner/rule/Passable.h>
USING_NAMESPACE_NOX;
namespace nox::app::rule
{
    void Passable::Apply(const PlannerBase::Frame & frame, ReferenceLine & referenceLine) const
    {
        if(not referenceLine.passable)
        {
            auto & planning_start_point = frame.stitch->Back();
            auto frenet = referenceLine.CalculateFrenet(planning_start_point.pose);
            double ds = referenceLine.Length() - frenet.s - planning_start_point.v * 8;

            referenceLine.Kill();

            if(ds < 50.0)
            {
                referenceLine.AddCost(ReferenceLine::Definite, 1);
                Logger::D("Rule") << "Passable rule kill reference line";
            }
            else
                referenceLine.AddCost(ReferenceLine::Available, 1);
        }
    }
}

