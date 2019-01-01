#include <utility>
#include <iostream>
#include <Planner/tool/BoundaryChecker.h>
#include <type/scene/Vehicle.h>

using namespace std;

namespace nox::app
{

    BoundaryChecker::BoundaryChecker(Ptr<ReferenceLine> reference, Ptr<Vehicle> vehicle)
        : _reference(std::move(reference)), _vehicle(std::move(vehicle))
    {}

    bool BoundaryChecker::Check(const type::Trajectory &trajectory)
    {
        std::vector<scene::ID> temp;
        return Check(trajectory, temp);
    }

    bool BoundaryChecker::Check(const type::Trajectory &trajectory, std::vector<scene::ID> &ids)
    {
        double half_width = _vehicle->param.length.y * 0.5;
        size_t nearest_index = trajectory.QueryNearestByPosition(_vehicle->pose.t);

        for(size_t idx = nearest_index, size = trajectory.Size(); idx < size; ++idx)
        {
            auto & i = trajectory[idx];
            auto frenet = _reference->CalculateFrenet(i.pose);
            auto b = _reference->GetBoundary(frenet.s);

            if(frenet.s < 0 or frenet.s > _reference->Length())
                continue;

            if(frenet.l + half_width > b.Upper)
                return false;

            if(frenet.l - half_width < b.Lower)
                return false;
        }

        return true;
    }

    bool BoundaryChecker::Check(const type::Trajectory &trajectory, double init_s)
    {
        double half_width = _vehicle->param.length.y * 0.5;
        size_t nearest_index = trajectory.QueryNearestByPosition(_vehicle->pose.t);

        for(size_t idx = nearest_index, size = trajectory.Size(); idx < size; ++idx)
        {
            auto & i = trajectory[idx];
            double s = i.s + init_s;
            auto p = _reference->path.PointAtDistance(s);
            double l = ((PathPoint)(i)).LateralTo(p);
            auto b = _reference->GetBoundary(s);

            if(p.s < 0 or p.s > _reference->Length())
                continue;

            if(l + half_width > b.Upper)
                return false;

            if(l - half_width < b.Lower)
                return false;
        }

        return true;
    }
}
