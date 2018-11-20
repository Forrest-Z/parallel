/**
 * @file Filter.h
 */

#pragma once

#include <nox>
#include <vector>

namespace nox::app
{
    /**
     * @class Filter
     * @brief 对一个数值进行滤波，注意，上一个Rule的输出为下一个Rule的输入
     */
    class Filter
    {
    public:
        class Rule
        {
        public:
            virtual double Apply(double raw) = 0;

            virtual ~Rule() = default;
        };

    public:
        template <class T, class ... Args>
        void AddRule(Args && ... args)
        {
            _rules.emplace_back(New<T>(args...));
        }

        double operator()(double raw);

    private:
        std::vector<Ptr<Rule>> _rules;
    };
}