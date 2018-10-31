/**
 * @brief 引导线规则：停止线：
 * 1. 太远不予以考虑；
 * 2. 接近时，惩罚有停止线的GuideLine
 */
#pragma once

#include <Planner/tool/GuideDecider.h>

namespace nox::app::rule
{
    class StopLine : public GuideDecider::Rule
    {
    public:
        void Apply(Ptr<GuideDecider> decider, Ptr<ReferenceLine> referenceLine) const override;
    };
}