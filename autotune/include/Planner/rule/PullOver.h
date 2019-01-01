/**
 * @file PullOver.h 靠边停车规则
 * 1. 仅选择passable的引导线进行判断；
 * 2. 优先选择最后一个点在前进方向右边的路线；
 * 3. 接近末端时才生效
 */

#pragma once

#include <Planner/tool/DecisionMaker.h>

namespace nox::app::rule
{
    class PullOver : public DecisionMaker::Rule
    {
    public:
        void Apply(const PlannerBase::Frame &frame, ReferenceLine &referenceLine) const override;
    };
}