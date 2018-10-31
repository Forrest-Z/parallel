#include <Planner/type/ReferenceLine.h>
#include <nox>
USING_NAMESPACE_NOX;
using namespace nox::app;


ReferenceLine::ReferenceLine(const GuideLine &guideLine)
    : GuideLine(guideLine)
{}

void ReferenceLine::AddCost(ReferenceLine::Priority priority, double cost)
{
    _priority[size_t(priority)] += cost;
}

bool ReferenceLine::IsPriorThan(const ReferenceLine &other) const
{
    for(size_t i = 0, size = _priority.size(); i < size; i++)
    {
        if(Real::IsEqual(_priority[i], other._priority[i]))
            continue;
        else
            return _priority[i] < other._priority[i];
    }

    return false;
}

bool ReferenceLine::IsReachedEnd(Ptr<type::Vehicle> vehicle) const
{
    auto nearest_index = path.QueryNearestByPosition(vehicle->pose.t);
    auto nearest_point = path[nearest_index];

    return (path.Back().s - nearest_point.s) < vehicle->param.length.x * 1.5;
}

math::Frenet ReferenceLine::CalculateFrenet(double x, double y, double theta) const
{
    auto frenet = path.FrenetAtPosition({x, y});
    frenet.theta = theta;
    return frenet;
}

math::Frenet ReferenceLine::CalculateFrenet(Ptr<type::Vehicle> vehicle) const
{
    assert(vehicle);
    return CalculateFrenet(vehicle->pose.x, vehicle->pose.y, vehicle->pose.theta);
}

double ReferenceLine::Length() const
{
    return path.Length();
}

double ReferenceLine::CruisingSpeed() const
{
    if(speedLimits.empty())
        return 5.0;
    else
        return speedLimits.front().v.Upper;
}

double ReferenceLine::StopPoint() const
{
    return stopLine.value_or(real::MAX).s;
}

