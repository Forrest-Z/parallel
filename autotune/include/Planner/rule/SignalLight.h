/**
 * @brief 交通规则：交通灯
 */
#pragma once

#include <Planner/tool/TrafficDecider.h>

namespace nox::app::rule
{
    class SignalLight : public TrafficDecider::Rule
    {
    public:
        void Apply(TrafficDecider *decider, ReferenceLine &referenceLine) const override;
    };
}