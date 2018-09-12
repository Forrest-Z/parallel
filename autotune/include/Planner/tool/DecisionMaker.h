/**
 * @brief 决策器基类，包含子类Rule
 */
#pragma once

#include <Planner/type/ReferenceLine.h>
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
            virtual void Apply(DecisionMaker * decider, ReferenceLine & referenceLine) const = 0;
            virtual ~Rule() = default;

        private:
            friend class DecisionMaker;
            DecisionMaker * _decider = nullptr;
        };

        using PtrRule = std::shared_ptr<Rule>;

        virtual ~DecisionMaker() = default;

    public:
        void Execute(vector<ReferenceLine> & referenceLines) const;

        void Execute(ReferenceLine & referenceLine) const;

        DecisionMaker * AddRule(PtrRule rule);

        template <class TRule, class ... Args>
        DecisionMaker * AddRule(Args && ... args)
        {
            return AddRule(std::make_shared<TRule>(std::forward(args)...));
        }

    private:
        vector<PtrRule> _rules;
    };
}