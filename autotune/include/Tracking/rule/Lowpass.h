/**
 * @file Lowpass.h
 */

#pragma once

#include <Tracking/tool/Filter.h>

namespace nox::app::rule
{
    class Lowpass
        : public Filter::Rule
    {
    public:
        Lowpass(size_t bufferSize, double init_data = 0);

        double Apply(double raw) override;

    private:
        container::Buffer<double> _buffer;
    };
}