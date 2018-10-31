/**
 * @brief 引导线规则：通过性
 * 1. 太远的时候，惩罚较小的代价
 * 2. 较近的时候，需要绝对禁止
 */
#pragma once

#include <Planner/tool/GuideDecider.h>

namespace nox::app::rule
{
    class Passable : public GuideDecider::Rule
    {
    public:
        void Apply(Ptr<GuideDecider> decider, Ptr<ReferenceLine> referenceLine) const override;
    };
}