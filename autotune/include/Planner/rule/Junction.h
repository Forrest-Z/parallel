/**
 * @brief 交通规则：路口
 * @details
 * 1. 接近路口时需要变道到应有的车道上；
 * 2. 如果判断的reference line不是目标车道，则：
 *   a. 接近路口时，直接封死
 *   b. 其他情况，优先级更低一点
 */
#pragma once

#include <Planner/tool/TrafficDecider.h>

namespace nox::app::rule
{
    class Junction : public TrafficDecider::Rule
    {
    public:
        void Apply(TrafficDecider *decider, ReferenceLine &referenceLine) const override;
    };
}