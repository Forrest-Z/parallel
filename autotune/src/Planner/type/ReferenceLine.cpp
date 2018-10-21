#include <Planner/type/ReferenceLine.h>
#include <nox>
USING_NAMESPACE_NOX;
using namespace nox::app;

ReferenceLine::ReferenceLine()
{
    for (double &i : _priority)
        i = 1;
    _drivable = true;
    _target.s = Real::MAX;
    _target.v = 0;
}

ReferenceLine::ReferenceLine(const type::Lane &lane)
    : ReferenceLine()
{
    path = AddressOf(lane.path);
    laneID = lane.id;
    width = 3.5; // TODO
    _target.v = lane.minSpeed / 3.6;
}


void ReferenceLine::AddCost(ReferenceLine::Priority priority, double cost)
{
    _priority[size_t(priority)] = cost;
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

void ReferenceLine::SetDrivable(bool drivable)
{
    _drivable = drivable;
}

void ReferenceLine::SetStopPoint(double s)
{
    _target.s = std::min(_target.s, s);
    _target.v = 0;
}

bool ReferenceLine::IsReachedEnd(Ptr<type::Vehicle> vehicle) const
{
    auto nearest_index = path->QueryNearestByPosition(vehicle->pose.t);
    auto nearest_point = path->at(nearest_index);

    return (path->Back().s - nearest_point.s) < vehicle->param.length.x * 1.5;
}

math::Frenet ReferenceLine::CalculateFrenet(double x, double y, double theta) const
{
    size_t nearest_index = path->QueryNearestByPosition({x, y});
    const auto nearest_point = path->at(nearest_index);

    Frenet frenet;
    math::Cartesian2Frenet(
        Cartesian(x, y, theta),
        Cartesian(nearest_point.pose.x, nearest_point.pose.y, nearest_point.pose.theta),
        nearest_point.s,
        frenet);

    return frenet;
}

ReferenceLine::Target ReferenceLine::GetTarget() const
{
    return _target;
}

bool ReferenceLine::Target::IsStop() const
{
    return v == 0;
}
