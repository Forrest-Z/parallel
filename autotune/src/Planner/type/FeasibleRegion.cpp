#include <Planner/type/FeasibleRegion.h>
#include <Planner/PlannerConfig.h>
#include <nox>

using namespace nox::app;


FeasibleRegion::FeasibleRegion(double s, double v, double a)
    : _s(s), _v(v), _a(a)
{
    auto vehicle = cache::ReadEgoVehicle();
    _longitudinal_acceleration.Upper = vehicle.param.limit.lon.a.Upper;
    _longitudinal_acceleration.Lower = vehicle.param.limit.lon.a.Lower;
    _base_speed = vehicle.param.baseSpeed;

    _comfort_lon_acceleration.Upper = 0.5 * vehicle.param.limit.lon.a.Upper;
    _comfort_lon_acceleration.Lower = 0.5 * vehicle.param.limit.lon.a.Lower;

    double max_deceleration = -_longitudinal_acceleration.Lower;
    _t_at_zero_speed = v / max_deceleration;
    _s_at_zero_speed = s + v * v / (2.0 * max_deceleration);
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
        return _base_speed;
}

double FeasibleRegion::TLower(double s) const
{
    assert(s >= _s);
    double ds = s - _s;
    double a = _longitudinal_acceleration.Upper;
    return (std::sqrt(_v * _v + 2.0 * a * ds) - _v) / a;
}

double FeasibleRegion::ComfortVUpper(double t) const
{
    return  _v + _comfort_lon_acceleration.Upper * t;
}

double FeasibleRegion::ComfortVLower(double t) const
{
    if(t < _t_at_zero_speed)
        return _v + _comfort_lon_acceleration.Lower * t;
    else
        return _base_speed;
}
