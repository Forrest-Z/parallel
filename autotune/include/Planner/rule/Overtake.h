/**
 * @brief 交通规则：变道超车
 */
#pragma once

#include <Planner/tool/TrafficDecider.h>

namespace nox::app::rule
{
    class Overtake : public TrafficDecider::Rule
    {
    public:
        void Apply(TrafficDecider *decider, ReferenceLine &referenceLine) const override;
    };
}
