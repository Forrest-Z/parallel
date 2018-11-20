/**
 * @brief 决策器基类，包含子类Rule
 */
#pragma once

#include <Planner/type/ReferenceLine.h>
#include <Planner/impl/PlannerBase.h>
#include <memory>
#include <vector>

namespace nox::app
{
    class DecisionMaker
    {
    public:
        class Rule
        {
        public:
            virtual void Apply(const PlannerBase::Frame & frame, ReferenceLine & referenceLine) const = 0;
            virtual ~Rule() = default;
        };

        virtual ~DecisionMaker() = default;

    public:
        void Execute(const PlannerBase::Frame & frame) const;

        void Execute(const PlannerBase::Frame & frame, ReferenceLine & referenceLine) const;

        Ptr<DecisionMaker> AddRule(Ptr<Rule> rule);

        template <class TRule, class ... Args>
        Ptr<DecisionMaker> AddRule(Args && ... args)
        {
            return AddRule(New<TRule>(std::forward(args)...));
        }

    private:
        vector<Ptr<Rule>> _rules;
    };
}