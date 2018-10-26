#include <Planner/type/PiecewiseJerkCurve.h>

using namespace nox::app;


PiecewiseJerkCurve::PiecewiseJerkCurve(double p, double v, double a)
{
    _last_p = p;
    _last_v = v;
    _last_a = a;
}

void PiecewiseJerkCurve::AppendSegment(double jerk, double param)
{
    auto segment = New<ConstantJerkCurve>(_last_p, _last_v, _last_a, jerk, param);
    Add(Knots().back() + param, segment);

    _last_p = segment->end_position();
    _last_v = segment->end_velocity();
    _last_a = segment->end_acceleration();
}
