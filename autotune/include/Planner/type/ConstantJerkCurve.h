//
// Created by yarten on 18-10-23.
//
#pragma once

#include <nox>

namespace nox::app
{
    class ConstantJerkCurve : public math::Parametric<1>
    {
    public:
        ConstantJerkCurve(
            double p0, double v0, double a0,
            double jerk, double t);

        double Calculate(size_t order, double t) const override;

        double Upper() const override;

        double start_position() const;

        double start_velocity() const;

        double start_acceleration() const;

        double end_position() const;

        double end_velocity() const;

        double end_acceleration() const;

        double jerk() const;

    private:
        double p0_;
        double v0_;
        double a0_;

        double p1_;
        double v1_;
        double a1_;

        double param_;

        double jerk_;
    };
}