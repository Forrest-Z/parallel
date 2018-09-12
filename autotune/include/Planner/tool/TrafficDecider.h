/**
 * @brief 交通规则决策器
 */
#pragma once

#include "DecisionMaker.h"
#include <nox>

namespace nox::app
{
    class TrafficDecider : public DecisionMaker
    {
    public:
        class Rule : public DecisionMaker::Rule
        {
        public:
            void Apply(DecisionMaker *decider, ReferenceLine &referenceLine) const final;
            virtual void Apply(TrafficDecider * decider, ReferenceLine & referenceLine) const = 0;
        };

    public:
        TrafficDecider(Ptr<type::Scene> & scene, Ptr<type::Vehicle> & vehicle);

    public:
        const Ptr<type::Vehicle> & vehicle;
        const Ptr<type::Scene> & scene;
    };
}