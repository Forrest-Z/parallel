/**
 * @brief 约束评估器，对一根曲线的检查
 */
#pragma once

#include <nox>

namespace nox::app
{
    class ConstraintChecker
    {
    public:
        ConstraintChecker(Ptr<type::Vehicle> vehicle);

        bool CheckLongitudinal(Ptr<math::Parametric<1>> lon_traj) const;

        bool CheckTrajectory(const type::Trajectory & trajectory) const;

    private:
        Ptr<type::Vehicle> _vehicle;

        struct
        {
            double _time_resolution = 0.1;
            double _planning_temporal_length = 8.0;
        } _param;
    };
}