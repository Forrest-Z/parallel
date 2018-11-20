#include <Planner/type/ConstantAccelerationCurve.h>
#include <math/geometry/Parametric.h>

namespace nox::app
{

    double ConstantAccelerationCurve::Calculate(size_t order, double t) const
    {
        assert(t >= 0 and t <= _t);
        switch (order)
        {
            case 0:
                return _s + _v * t + 0.5 * _a * t * t;
            case 1:
                return _v + _a * t;
            case 2:
                return _a;
            default:
                return 0;
        }
    }

    double ConstantAccelerationCurve::Boundary() const
    {
        return _t;
    }

    ConstantAccelerationCurve::ConstantAccelerationCurve(double s0, double v0, double t, double a)
        : _s(s0), _v(v0), _t(t), _a(a)
    {}
}
