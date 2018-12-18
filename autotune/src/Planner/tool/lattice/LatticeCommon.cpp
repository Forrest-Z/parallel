#include <utility>

#include <Planner/tool/lattice/LatticeCommon.h>
USING_NAMESPACE_NOX;
using namespace nox::app::lattice;


State::State()
    : math::Derivative<2>{0, 0, 0}, t(0)
{
}

State::State(const math::Derivative<2> &state, double t, double priority_factor)
    : math::Derivative<2>(state), t(t)
{
    cost_factor.all = priority_factor;
}

State::State(double s0, double s1, double s2, double t, double priority_factor)
    : State({s0, s1, s2}, t, priority_factor)
{}


Curve::Curve(nox::Ptr<nox::math::Parametric<1>> curve)
    : _curve(std::move(curve))
{}

double Curve::Calculate(size_t order, double param) const
{
    double boundary = _curve->Upper();
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

double Curve::Upper() const
{
    return _curve->Upper();
}

Combination::Combination(nox::Ptr<Curve> lon, nox::Ptr<Curve> lat)
    : lon(lon), lat(lat)
{

}

bool Combination::operator<(const Combination &other) const
{
    return cost_sum > other.cost_sum;
}
