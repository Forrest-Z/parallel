#include <Planner/rule/SignalLight.h>

using namespace nox::app;
using namespace nox::app::rule;


void SignalLight::Apply(TrafficDecider *decider, ReferenceLine &referenceLine) const
{
    if(!referenceLine.path) return;

    auto lane = decider->scene->GetLaneByID(referenceLine.laneID);
    auto next_junction = decider->scene->NextJunction();

    if(!next_junction) return; // 没有路口时，不进行红绿灯决策

    for(auto & it : decider->scene->signalLights)
    {
        auto i = it.second;
        /// 当红绿灯位于下一个路口时，才进行决策
        if(i->junctionID == next_junction->id)
        {
            /// 引导线是否位于进入路口的目标车道线
            bool is_junction_in_lane = next_junction ? next_junction->from == referenceLine.laneID : false;
            /// 引导线是否位于进入路口的车道区域
            bool is_junction_in_segment = decider->scene->IsInSameLaneSegment(next_junction->from, referenceLine.laneID);

            if(is_junction_in_segment)
            {
                if(is_junction_in_lane)
                {
                    /// 目前假设道路只有一个方向（由全局规划来负责）
                    if(!(lane->direction & i->direction))
                    {
                        /// 当灯为黄灯时，且车子已经接近车道线末端，则一脚油门带走
                        if(i->color == scene::Red or (i->color == scene::Yellow and !referenceLine.IsReachedEnd(decider->vehicle) ))
                            referenceLine.SetStopPoint(referenceLine.path->Back().s);
                    }
                }
                else /// 若引导线不是进入路口的车道线，则将设置车道线末端为停止点
                {
                    referenceLine.SetStopPoint(referenceLine.path->Back().s);
                }
            }
        }
    }
}
