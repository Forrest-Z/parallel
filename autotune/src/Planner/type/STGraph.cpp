#include <Planner/type/STGraph.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


STGraph::STGraph(
    Ptr<type::Scene> scene,
    Ptr<ReferenceLine> referenceLine,
    double start_s, double end_s,
    double start_t, double end_t,
    double path_width,
    double time_resolution)
{
    _s.Start = start_s;
    _s.End = end_s;
    _t.Start = start_t;
    _t.End = end_t;
    _time_resolution = time_resolution;
    _half_path_width = 0.5 * path_width;
    _reference = referenceLine;

    SetupObstacles(scene);
}

void STGraph::SetupObstacles(Ptr<type::Scene> scene)
{
    for(auto & it : scene->Obstacles)
    {
        auto & i = *it.second;
        if(i.IsStatic())
            AddStaticObstacle(i);
        else
            AddDynamicObstacle(i);
    }
}

void STGraph::AddStaticObstacle(const nox::type::Obstacle &obstacle)
{
    auto box = obstacle.BoxAtTime(0);
    auto boundary = ComputeObstacleBoundary(box);

    if(ShouldIgnore(boundary)) /// 当障碍物不在参考线范围内时，不添加。
        return;

    boundary.t.Start = 0;
    boundary.t.End = _t.End;

    auto st_obstacle = New<STObstacle>();
    st_obstacle->trajectory.emplace_back(boundary.t.Start, boundary.s);
    st_obstacle->trajectory.emplace_back(boundary.t.End, boundary.s);
    _obstacles[obstacle.id] = st_obstacle;
}

SLTBoundary STGraph::ComputeObstacleBoundary(const nox::type::Box &box) const
{
    SLTBoundary boundary;
    boundary.s.Start = type::Real::MAX;
    boundary.s.End   = -type::Real::MAX;
    boundary.l.Start = type::Real::MAX;
    boundary.l.End   = -type::Real::MAX;

    for(auto & i : box.Corners())
    {
        auto frenet = _reference->CalculateFrenet(i.x, i.y, 0);
        boundary.s.Start = std::min(boundary.s.Start, frenet.s);
        boundary.s.End   = std::max(boundary.s.End, frenet.s);
        boundary.l.Start = std::min(boundary.l.Start, frenet.l);
        boundary.l.End   = std::max(boundary.l.End, frenet.l);
    }

    return boundary;
}

void STGraph::AddDynamicObstacle(const nox::type::Obstacle &obstacle)
{
    for (double time = _t.Start; time < _t.End; time += _time_resolution)
    {
        auto box = obstacle.BoxAtTime(time);
        auto boundary = ComputeObstacleBoundary(box);

        if (ShouldIgnore(boundary))   /// 当障碍物当前位置的包围范围超出了path-time图的范围，则不操作
        {
            continue;
        }

        auto id = obstacle.id;
        if(auto it = _obstacles.find(id); it == _obstacles.end())
        {
            auto st_obstacle = New<STObstacle>();
            st_obstacle->trajectory.emplace_back(time, boundary.s);
            _obstacles[id] = st_obstacle;
        }
        else
        {
            it->second->trajectory.emplace_back(time, boundary.s);
        }
    }
}

nox::Ptr<STObstacle> STGraph::GetObstacle(scene::ID id)
{
    return _obstacles[id];
}

double STGraph::TimeResolution() const
{
    return _time_resolution;
}

const nox::type::Range &STGraph::SRange() const
{
    return _s;
}

const nox::type::Range &STGraph::TRange() const
{
    return _t;
}

vector<nox::type::Bound> STGraph::GetLateralBounds(double s_start, double s_end, double s_resolution)
{
    vector<nox::type::Bound> bounds;



    return bounds;
}

bool STGraph::ShouldIgnore(const SLTBoundary &boundary) const
{
    return boundary.s.End < _s.Start or
           boundary.s.Start > _s.End or
           boundary.l.Start > _half_path_width or
           boundary.l.End < -_half_path_width;
}

STPoint::STPoint(double t, double s_begin, double s_end)
{
    this->t = t;
    this->s.Start = s_begin;
    this->s.End = s_end;
}

STPoint::STPoint(double t, nox::type::Range s)
    : t(t), s(s)
{}

struct STPointComparator
{
    bool operator()(const STPoint & a, const STPoint & b)
    {
        return a.t < b.t;
    }
};

nox::type::Range STObstacle::EstimateAtTime(double t) const
{
    STPoint target;
    target.t = t;

    auto it = std::lower_bound(trajectory.begin(), trajectory.end(), target, STPointComparator());
    size_t index = std::distance(trajectory.begin(), it);

    if(index == trajectory.size() - 1)
        return trajectory.back().s;
    else
    {
        auto & p0 = trajectory[index];
        auto & p1 = trajectory[index + 1];

        type::Range s_range;
        s_range.Start = math::LinearInterpolate(p0.s.Start, p1.s.Start, p0.t, p1.t, t);
        s_range.End = math::LinearInterpolate(p0.s.End, p1.s.End, p0.t, p1.t, t);

        return s_range;
    }
}
