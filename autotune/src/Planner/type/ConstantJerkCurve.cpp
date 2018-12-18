#include <Planner/type/ConstantJerkCurve.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


ConstantJerkCurve::ConstantJerkCurve(
    const double p0, const double v0, const double a0,
    const double j, const double param)
    : p0_(p0), v0_(v0), a0_(a0), param_(param), jerk_(j)
{
    assert(param > real::Epsilon);
    p1_ = Calculate(0, param_);
    v1_ = Calculate(1, param_);
    a1_ = Calculate(2, param_);
}

double ConstantJerkCurve::Calculate(size_t order, double param) const
{
    assert(param > -real::Epsilon);
    switch (order)
    {
        case 0:
        {
            return p0_ + v0_ * param + 0.5 * a0_ * param * param +
                   jerk_ * param * param * param / 6.0;
        }
        case 1:
        {
            return v0_ + a0_ * param + 0.5 * jerk_ * param * param;
        }
        case 2:
        {
            return a0_ + jerk_ * param;
        }
        case 3:
        {
            return jerk_;
        }
        default:
            return 0.0;
    }
}

double ConstantJerkCurve::start_position() const
{
    return p0_;
}

double ConstantJerkCurve::start_velocity() const
{
    return v0_;
}

double ConstantJerkCurve::start_acceleration() const
{
    return a0_;
}

double ConstantJerkCurve::end_position() const
{
    return p1_;
}

double ConstantJerkCurve::end_velocity() const
{
    return v1_;
}

double ConstantJerkCurve::end_acceleration() const
{
    return a1_;
}

double ConstantJerkCurve::Upper() const
{
    return param_;
}

double ConstantJerkCurve::jerk() const
{
    return jerk_;
}