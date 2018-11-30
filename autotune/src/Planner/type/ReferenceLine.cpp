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

math::Frenet ReferenceLine::CalculateFrenet(const Pose &pose) const
{
    return CalculateFrenet(pose.x, pose.y, pose.theta);
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
    return CalculateFrenet(vehicle->pose);
}

double ReferenceLine::Length() const
{
    return path.Length();
}

double ReferenceLine::CruisingSpeed() const
{
    return 20.0 / 3.6;
    if(speedLimits.empty())
        return 5.0;
    else
        return speedLimits.front().v.Upper;
}

double ReferenceLine::StopPoint() const
{
    return stopLine.value_or(real::MAX).s;
}

void ReferenceLine::Kill()
{
    _killed = true;
}

bool ReferenceLine::Dead() const
{
    return _killed;
}

bool ReferenceLine::IsBeyondStopLine(const Pose &pose) const
{
    if(not stopLine)
        return false;
    else
    {
        auto frenet = CalculateFrenet(pose);
        return frenet.s > stopLine.value().s;
    }
}


