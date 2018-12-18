#include <Planner/type/PiecewiseAccelerationCurve.h>

using namespace nox::app;


PiecewiseAccelerationCurve::PiecewiseAccelerationCurve(double start_s, double start_v)
    : _s0(start_s), _v0(start_v)
{}

void PiecewiseAccelerationCurve::PushSegment(double a, double dt)
{
    double s0 = _s0;
    double v0 = _v0;
    double t0 = Knots().back();

    size_t segment_size = SegmentsCount();
    if(segment_size != 0)
    {
        auto segment = GetSegment(segment_size - 1);
        double dt0 = segment->Upper();
        s0 = segment->Calculate(0, dt0);
        v0 = segment->Calculate(1, dt0);
    }

    Add(t0 + dt, New<ConstantAccelerationCurve>(s0, v0, dt, a));
}
