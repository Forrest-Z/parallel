/**
 * @file Custom.h
 */

#pragma once

#include <Tracking/tool/Filter.h>
#include <functional>

namespace nox::app::rule
{
    class Custom
        : public Filter::Rule
    {
        using TFunction = std::function<double(double raw)>;
    public:
        Custom() = default;

        Custom(const TFunction & func);

        double Apply(double raw) override;

    private:
        TFunction _func;
    };
}