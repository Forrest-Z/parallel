#include <Planner/type/FeasibleRegion.h>
#include <nox>

using namespace nox::app;


FeasibleRegion::FeasibleRegion(double s, double v, double a)
    : _s(s), _v(v), _a(a)
{
    auto vehicle = type::Vehicle::Instance();
    _longitudinal_acceleration.Upper = vehicle->param.limit.lon.a.Upper;
    _longitudinal_acceleration.Lower = vehicle->param.limit.lon.a.Lower;

    double max_deceleration = -_longitudinal_acceleration.Lower;
    _t_at_zero_speed = v / max_deceleration;
    _t_at_zero_speed = s + v * v / (2.0 * max_deceleration);
}

double FeasibleRegion::SUpper(double t) const
{
    assert(t >= 0.0);
    return _s + _v * t + 0.5 * _longitudinal_acceleration.Upper * t * t;
}

double FeasibleRegion::SLower(double t) const
{
    if(t < _t_at_zero_speed)
        return _s + _v * t + 0.5 * _longitudinal_acceleration.Lower * t * t;
    else
        return _s_at_zero_speed;
}

double FeasibleRegion::VUpper(double t) const
{
    return _v + _longitudinal_acceleration.Upper * t;
}

double FeasibleRegion::VLower(double t) const
{
    if(t < _t_at_zero_speed)
        return _v + _longitudinal_acceleration.Lower * t;
    else
        return 0;
}

double FeasibleRegion::TLower(double s) const
{
    assert(s >= _s);
    double ds = s - _s;
    double a = _longitudinal_acceleration.Upper;
    return (std::sqrt(_v * _v + 2.0 * a * ds) - _v) / a;
}
