#include <Planner/type/ReferenceLine.h>
#include <nox>
USING_NAMESPACE_NOX;
using namespace nox::app;

ReferenceLine::ReferenceLine()
{
    for (double &i : _priority)
        i = 1;
    _drivable = true;
    _stopPoint = Real::MAX;
}

void ReferenceLine::AddCost(ReferenceLine::Priority priority, double cost)
{
    _priority[size_t(priority)] = cost;
}

bool ReferenceLine::PriorThan(const ReferenceLine &other) const
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
    _stopPoint = std::min(_stopPoint, s);
}

bool ReferenceLine::IsReachedEnd(Ptr<type::Vehicle> vehicle) const
{
    // TODO: 通过车的大小、以及相关参数进行判断
    return false;
}


