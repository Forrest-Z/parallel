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
            virtual void Apply(Ptr<DecisionMaker> decider, Ptr<ReferenceLine> referenceLine) const = 0;
            virtual ~Rule() = default;

        private:
            friend class DecisionMaker;
            Ptr<DecisionMaker> _decider;
        };

        virtual ~DecisionMaker() = default;

    public:
        void Execute(vector<Ptr<ReferenceLine>> & referenceLines) const;

        void Execute(Ptr<ReferenceLine> referenceLine) const;

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