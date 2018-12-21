#include <Planner/type/QuadraticAccelerationCurve.h>
#include <math/geometry/Parametric.h>

namespace nox::app
{

    QuadraticAccelerationCurve::QuadraticAccelerationCurve(
        const math::Derivative<2> &s0,
        const math::Derivative<2> &s1,
        double t)
        : x0(s0[0]), v0(s0[1]), a0(s0[2]), t(t), k1(0), k2(0)
    {
        if(t == 0) return;

        double t_2 = t * t;
        double t_3 = t_2 * t;

        double A = t;
        double B = t_2;
        double X = s1[2] - a0;

        double C = 0.5 * t_2;
        double D = 1.0 / 3.0 * t_3;
        double Y = s1[1] - s1[2] * t - v0;

        double BC_AD = B * C - A * D;
        k1 = (B*Y - D*X) / BC_AD;
        k2 = (X*C - A*Y) / BC_AD;

        math::Polynomial polynomial;
        polynomial.SetCoefficient({x0, v0, 0.5*a0, 1.0/6.0*k1, 1.0/12.0*k2});
        polynomial.SetUpper(t);
        SetPolynomial(polynomial);
    }
}