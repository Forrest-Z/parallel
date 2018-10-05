#include <Planner/type/ConstantAccelerationCurve.h>

using namespace nox::app;


ConstantAccelerationCurve::ConstantAccelerationCurve(double start_s, double start_v)
{
    _s.push_back(start_s);
    _v.push_back(start_v);
    _t.push_back(0);
    _a.push_back(0);
}

void ConstantAccelerationCurve::PushSegment(double a, double dt)
{
    double s0 = _s.back();
    double v0 = _v.back();
    double t0 = _t.back();

    double v1 = v0 + a * dt;

    double ds = (v0 + v1) * dt * 0.5;
    double s1 = s0 + ds;
    double t1 = t0 + dt;
    
    s1 = std::max(s1, s0);
    _s.push_back(s1);
    _v.push_back(v1);
    _a.push_back(a);
    _t.push_back(t1);
}

void ConstantAccelerationCurve::PopSegment()
{
    if (!_a.empty()) 
    {
        _s.pop_back();
        _v.pop_back();
        _a.pop_back();
        _t.pop_back();
    }
}

double ConstantAccelerationCurve::Calculate(size_t order, double param) const
{
    assert(_t.size() > 1);
    assert(_t.front() <= param && param <= _t.back());

    switch (order)
    {
        case 0:
            return CalculateS(param);
        case 1:
            return CalculateV(param);
        case 2:
            return CalculateA(param);
        default:
            return 0;
    }
}

double ConstantAccelerationCurve::Boundary() const
{
    return _t.back() - _t.front();
}

double ConstantAccelerationCurve::CalculateS(double t) const
{
    auto it_lower = std::lower_bound(_t.begin(), _t.end(), t);
    auto index = std::distance(_t.begin(), it_lower);

    double s0 = _s[index];
    double v0 = _v[index];
    double t0 = _t[index];

    if(index == _v.size() - 1)
    {
        return s0;
    }
    else
    {
        double v1 = _v[index + 1];
        double t1 = _t[index + 1];

        double v = math::LinearInterpolate(v0, v1, t0, t1, t);
        double s = (v0 + v) * (t - t0) * 0.5 + s0;
        return s;
    }
}

double ConstantAccelerationCurve::CalculateV(double t) const
{
    auto it_lower = std::lower_bound(_t.begin(), _t.end(), t);
    auto index = std::distance(_t.begin(), it_lower);

    double v0 = _v[index];
    double t0 = _t[index];

    if(index == _v.size() - 1)
    {
        return v0;
    }
    else
    {
        double v1 = _v[index + 1];
        double t1 = _t[index + 1];

        double v = math::LinearInterpolate(v0, v1, t0, t1, t);
        return v;
    }
}

double ConstantAccelerationCurve::CalculateA(double t) const
{
    auto it_lower = std::lower_bound(_t.begin(), _t.end(), t);
    auto index = std::distance(_t.begin(), it_lower);
    return _a[index - 1];
}

