#include <Planner/tool/lattice/LatticeCommon.h>

using namespace nox::app::lattice;


State::State()
    : math::Derivative<3>{0, 0, 0}, t(0)
{
}

State::State(const math::Derivative<3> &state, double t)
    : math::Derivative<3>(state), t(t)
{}

State::State(double s0, double s1, double s2, double t)
    : math::Derivative<3>{s0, s1, s2}, t(t)
{}

Curve::Curve(nox::Ptr<nox::math::Parametric> curve)
    : _curve(curve)
{}

double Curve::Calculate(size_t order, double param) const
{
    double boundary = _curve->Boundary();
    if(param < boundary)
    {
        return _curve->Calculate(order, param);
    }

    double p = _curve->Calculate(0, boundary);
    double v = _curve->Calculate(1, boundary);
    double a = _curve->Calculate(2, boundary);
    double t = param - boundary;

    switch (order)
    {
        case 0:
            return p + v * t + 0.5 * a * t * t;
        case 1:
            return v + a * t;
        case 2:
            return a;
        default:
            return 0.0;
    }
}

double Curve::Boundary() const
{
    return _curve->Boundary();
}

Combination::Combination(nox::Ptr<nox::math::Parametric> lon, nox::Ptr<nox::math::Parametric> lat)
    : lon(lon), lat(lat)
{

}

bool Combination::operator<(const Combination &other) const
{
    return cost_sum < other.cost_sum;
}


