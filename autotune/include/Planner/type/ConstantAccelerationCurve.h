/**
 * @file ConstantAccelerationCurve.h
 */

#pragma once

#include <nox>

namespace nox::app
{
    class ConstantAccelerationCurve
        : public math::Parametric<1>
    {
    public:
        ConstantAccelerationCurve() = default;

        ConstantAccelerationCurve(double s0, double v0, double t, double a);

        double Calculate(size_t order, double t) const override;

        double Upper() const override;

    private:
        double _s = 0;
        double _v = 0;
        double _t = 0;
        double _a = 0;
    };
}