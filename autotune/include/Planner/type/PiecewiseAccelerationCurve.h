/**
 * @brief 描述恒定加速的的轨迹
 */
#pragma once

#include <nox>
#include <vector>
#include <Planner/type/ConstantAccelerationCurve.h>

namespace nox::app
{
    class PiecewiseAccelerationCurve :
        public math::Piecewise<ConstantAccelerationCurve>
    {
    public:
        PiecewiseAccelerationCurve(double start_s, double start_v);

        void PushSegment(double a, double dt);

    private:
        double _s0 = 0;
        double _v0 = 0;
    };
}
