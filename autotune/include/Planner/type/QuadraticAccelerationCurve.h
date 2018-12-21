/**
 * @file QuadraticAccelerationCurve.h，拥有二次型的加速度曲线，
 * 使用初始和终止两个状态进行计算得到，两个状态间的时间长度是估算得到的。
 */

#pragma once

#include <nox>

namespace nox::app
{
    class QuadraticAccelerationCurve :
        public math::FastPolynomial
    {
    public:
        QuadraticAccelerationCurve(
            const math::Derivative<2> & s0,
            const math::Derivative<2> & s1,
            double t
        );


    private:
        double a0, v0, x0;
        double k1, k2, t;
    };
}