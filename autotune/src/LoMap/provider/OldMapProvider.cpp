#include <LoMap/provider/OldMapProvider.h>
#include <LoMap/tool/GuideLineBuilder.h>
#include <Planner/PlannerConfig.h>

USING_NAMESPACE_NOX;

namespace nox::app
{

    void OldMapProvider::Update(const nox_msgs::Road &road)
    {
        MD5<nox_msgs::Road> road_(road);
        road_.md5() = Clock::us();
        _old_map.Update(road_);
    }

    MD5<vector<Ptr<nox::type::GuideLine>>> OldMapProvider::Produce()
    {
        if(not _old_map.IsFresh())
            return _guideLines;

        auto old_map = _old_map.Get();
        auto road = old_map.data();
        _guideLines.reset({}, old_map.md5());

        auto guideLine = New<GuideLine>();
        vector<AnchorPoint> anchors;
        vector<SpeedSegment> speeds;

        for(auto & i : road.blocks)
        {
            for(auto & j : i.points)
            {
                AnchorPoint anchor;
                anchor.pose.x = j.x;
                anchor.pose.y = j.y;
                anchor.pose.theta = (j.yaw + 90) / 180.0 * M_PI;
                anchor.lateralBound = 0.1;
                anchor.longitudinalBound = 2;

                if(not anchors.empty())
                    anchor.s = anchors.back().s + anchors.back().pose.t.DistanceTo(anchor.pose.t);
                anchors.push_back(anchor);

                if(speeds.empty() or speeds.back().speed.Upper != i.maxSpeed)
                {
                    speeds.emplace_back();
                    speeds.back().begin.x = j.x;
                    speeds.back().begin.y = j.y;
                    speeds.back().speed.Upper = i.maxSpeed;
                }

                speeds.back().end.x = j.x;
                speeds.back().end.y = j.y;
                break;
            }
        }

        GuideLineBuilder::BuildPathUsingSpline(anchors, *guideLine);
        AddSpeedControl(speeds, *guideLine);
        AddDeadEnd(*guideLine);
//        AddBoundary(*guideLine);
        _guideLines.data().push_back(guideLine);
        return _guideLines;
    }

    void
    OldMapProvider::AddSpeedControl(const vector<OldMapProvider::SpeedSegment> &speeds, GuideLine &guideLine) const
    {
        for(auto & i : speeds)
        {
            auto f0 = guideLine.path.FrenetAtPosition(i.begin);
            auto f1 = guideLine.path.FrenetAtPosition(i.end);
            guideLine.speed.Add(key::RoadRule, f0.s, f1.s, 0, i.speed.Upper / 3.6);
        }
    }

    void OldMapProvider::AddDeadEnd(GuideLine &guideLine) const
    {
        guideLine.stop.Add(key::DeadEnd, guideLine.path.Back().s);
    }

    void OldMapProvider::AddBoundary(GuideLine &guideLine) const
    {
        Boundary upper_bound, lower_bound;
        double s = guideLine.path.Back().s;
        const double half_width = 3.5 * 0.5;

        upper_bound.func.x = upper_bound.s =
        lower_bound.func.x = lower_bound.s =
        Bound(0, s);

        upper_bound.passable = lower_bound.passable = false;
        upper_bound.func.type = lower_bound.func.type = Function::Polynomial;

        upper_bound.func.coeff =
        math::Polynomial(vector<double>{+half_width}, s, 0).Coefficient();

        lower_bound.func.coeff =
        math::Polynomial(vector<double>{-half_width}, s, 0).Coefficient();

        guideLine.boundary.Add(key::MapEdge, upper_bound);
        guideLine.boundary.Add(key::MapEdge, lower_bound);
    }
}
