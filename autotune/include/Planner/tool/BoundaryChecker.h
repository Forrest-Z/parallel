/**
 * @file BoundaryChecker.h
 */

#pragma once

#include <nox>
#include <Planner/type/ReferenceLine.h>

namespace nox::app
{
    class BoundaryChecker
    {
    public:
        BoundaryChecker(Ptr<ReferenceLine> reference, Ptr<Vehicle> vehicle);

        bool Check(const type::Trajectory & trajectory);

        bool Check(const type::Trajectory & trajectory, std::vector<scene::ID> & ids);

        bool Check(const type::Trajectory & trajectory, double init_s);

    private:
        Ptr<ReferenceLine> _reference;
        Ptr<Vehicle>       _vehicle;
    };
}