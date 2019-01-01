#include <Planner/rule/PullOver.h>
USING_NAMESPACE_NOX;

namespace nox::app::rule
{

    void PullOver::Apply(const PlannerBase::Frame &frame, ReferenceLine &referenceLine) const
    {
        if(referenceLine.passable)
        {
            auto & planning_start_point = frame.stitch->Back();
            auto frenet = referenceLine.CalculateFrenet(planning_start_point.pose);
            double ds = referenceLine.Length() - frenet.s - planning_start_point.v * 8;

//            if(ds > 50.0) return;

            auto & back = referenceLine.path.Back();
            bool rightmost = true;

            for(auto & i : frame.references)
            {
                if(not i->passable) continue;
                if(back.LateralTo(i->path.Back()) > real::Epsilon)
                    rightmost = false;
            }

            if(not rightmost)
            {
                referenceLine.Kill();
                Logger::D("Rule") << "PullOver rule kill reference line";
            }
        }
    }
}
