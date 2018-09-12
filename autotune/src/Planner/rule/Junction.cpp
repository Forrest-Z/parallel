#include <Planner/rule/Junction.h>
using namespace nox::app::rule;
using namespace nox::app;


void Junction::Apply(TrafficDecider *decider, ReferenceLine &referenceLine) const
{
    if(!referenceLine.path) return;

    auto next_junction = decider->scene->NextJunction();
    auto nearest_index = referenceLine.path->QueryNearest(decider->vehicle->position());
    auto nearest_point = referenceLine.path->operator[](nearest_index);

    double s = nearest_point.LongitudinalFrom(referenceLine.path->Back());
    double threshold = std::max(15.0, decider->vehicle->vx() * 3.0);

    /// 车是否位于车道线末端
    bool far_away_from_junction = s > threshold;
    /// 引导线是否位于进入路口的目标车道线
    bool is_junction_in_lane = next_junction ? next_junction->from == referenceLine.laneID : false;
    /// 引导线是否位于进入路口的车道区域
    bool is_junction_in_segment = decider->scene->IsInSameLaneSegment(next_junction->from, referenceLine.laneID);

    /// 当引导线位于进入路口的车道区域中时，本决策才有意义
    if(is_junction_in_segment)
    {
        if(far_away_from_junction)
        {
            if(!is_junction_in_lane)
                referenceLine.AddCost(ReferenceLine::Priority::_2, 1);
        }
        else
        {
            if(is_junction_in_lane)
                referenceLine.AddCost(ReferenceLine::Priority::_0, -1);
            else
                referenceLine.SetDrivable(false);
        }
    }
}
