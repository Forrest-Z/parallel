//
// Created by yarten on 18-10-23.
//
#pragma once

#include <nox>
#include "ConstantJerkCurve.h"

namespace nox::app
{
    class PiecewiseJerkCurve : public math::Piecewise<ConstantJerkCurve>
    {
    public:
        PiecewiseJerkCurve(double p, double v, double a);

        void AppendSegment(double jerk, double param);

    private:
        double _last_p;
        double _last_v;
        double _last_a;
    };
}