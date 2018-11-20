/**
 * @file Limiting.h
 */

#pragma once

#include <Tracking/tool/Filter.h>

namespace nox::app::rule
{
    class Limiting
        : public Filter::Rule
    {
    public:
        Limiting() = default;

        Limiting(double min, double max);

        double Apply(double raw) override;

    private:
        double _max = real::MAX;
        double _min = -real::MAX;
    };
}