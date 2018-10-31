/**
 * @brief 对ReferenceLine的引导信息，根据当前的车体状态做决策
 */
#pragma once

#include "DecisionMaker.h"

namespace nox::app
{
    class GuideDecider : public DecisionMaker
    {
    public:
        class Rule : public DecisionMaker::Rule
        {
        public:
            void Apply(Ptr<DecisionMaker> decider, Ptr<ReferenceLine> referenceLine) const final;

            virtual void Apply(Ptr<GuideDecider> decider, Ptr<ReferenceLine> referenceLine) const = 0;
        };

    public:
        explicit GuideDecider(Ptr<Vehicle> vehicle);

        Ptr<Vehicle> vehicle;
    };
}
