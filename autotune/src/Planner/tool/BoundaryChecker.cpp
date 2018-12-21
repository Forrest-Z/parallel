#include <utility>

#include <Planner/tool/BoundaryChecker.h>
#include <type/scene/Vehicle.h>

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

        for(auto & i : trajectory)
        {
            auto frenet = _reference->CalculateFrenet(i.pose);
            auto b = _reference->GetBoundary(frenet.s);

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

        for(auto & i : trajectory)
        {
            double s = i.s + init_s;
            auto p = _reference->path.PointAtDistance(s);
            double l = p.pose.t.DistanceTo(i.pose.t);
            auto b = _reference->GetBoundary(s);

            if(l + half_width > b.Upper)
                return false;
            if(l - half_width < b.Lower)
                return false;
        }

        return true;
    }
}
