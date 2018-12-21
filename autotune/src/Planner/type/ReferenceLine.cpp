#include <Planner/type/ReferenceLine.h>
#include <nox>
#include <cmath>
#include <Planner/PlannerConfig.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


ReferenceLine::ReferenceLine(const GuideLine &guideLine)
    : GuideLine(guideLine)
{
    Setup();
}

ReferenceLine &ReferenceLine::operator=(const GuideLine &guideLine)
{
    GuideLine::operator=(guideLine);
    Setup();
    return *this;
}


void ReferenceLine::Setup()
{
    //region 重置所有成员
    _priority = {1, 1, 1, 1};
    _killed = false;
    _stop_lines.clear();
    _speed_controls.Clear();
    _boundaries.Clear();
    //endregion

    //region 处理停止线
    for(auto & i : stop)
    {
        for(auto & j : i.data)
        {
            _stop_lines.push_back(j);
        }
    }

    std::sort(_stop_lines.begin(), _stop_lines.end());
    _stop_lines.emplace_back(Length() * 10);
    //endregion

    //region 处理速度信息
    for(auto & i : speed)
    {
        for(auto & j : i.data)
        {
            _speed_controls.PushBack(j.s, j.v);
        }
    }
    //endregion

    //region 处理边界信息
    for(auto & i : boundary)
    {
        for(auto & j : i.data)
        {
            _boundaries.PushBack(j.s.Lower, j.s.Upper, j.func.Generate());
        }
    }
    //endregion
}

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

bool ReferenceLine::IsReachedEnd(const Vehicle & vehicle) const
{
    auto nearest_index = path.QueryNearestByPosition(vehicle.pose.t);
    auto nearest_point = path[nearest_index];

    return (path.Back().s - nearest_point.s) < vehicle.param.length.x;
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

math::Frenet ReferenceLine::CalculateFrenet(const Vehicle & vehicle) const
{
    return CalculateFrenet(vehicle.pose);
}

double ReferenceLine::Length() const
{
    return path.Length();
}

double ReferenceLine::CruisingSpeed(double s) const
{
    double v_max = 0;

    for(auto & i : _speed_controls[s])
    {
        v_max = std::max<double>(v_max, i.data.Upper);
    }

    return v_max == 0 ? 20.0 / 3.6 : v_max;
}

void ReferenceLine::Kill()
{
    _killed = true;
}

bool ReferenceLine::IsDead() const
{
    return _killed;
}

Result<bool> ReferenceLine::IsNormal(const Trajectory &trajectory, const Vehicle &vehicle) const
{
    auto frenet = CalculateFrenet(vehicle);

    if(IsOverStopLine(frenet.s, vehicle.param.length.x))
        return Result(false, "Trajectory is over the stop line");

    double stop_line = GetNextStopLine(frenet.s);
    if(trajectory.Back().s >= stop_line)
        return Result(false, "Trajectory cross the stop line.");

    return true;
}

double ReferenceLine::GetNextStopLine(double s) const
{
    size_t index = math::UpperBound(
        _stop_lines.begin(),
        _stop_lines.end(),
        s,
        [](double s_, const StopLine & o)
        {
            return s_ < o.s;
        }
    );

    return _stop_lines[index].s;
}

bool ReferenceLine::IsOverStopLine(double s, double th) const
{
    size_t index = math::BinaryMatch(
        _stop_lines.begin(),
        _stop_lines.end(),
        s,
        [](const StopLine & o, double s_)
        {
            return o.s < s_;
        },
        [](const StopLine & lower, const StopLine & upper, double s_)
        {
            return s_ - lower.s < upper.s - s_;
        }
    );

    return abs(_stop_lines[index].s - s) < th;
}

Bound ReferenceLine::GetBoundary(double s) const
{
    Bound b(-real::MAX, real::MAX);

    for(auto & i : _boundaries[s])
    {
        double w = i.data->Calculate(0, s);
        if(w > 0)
            b.Upper = std::min<double>(b.Upper, w);
        else
            b.Lower = std::max<double>(b.Lower, w);
    }

    return b;
}





