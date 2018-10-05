/**
 * @brief 局部规划基类
 */
#pragma once

#include <Planner/type/ReferenceLine.h>
#include <nox>
#include <vector>

namespace nox::app
{
    class PlannerBase
    {
    public:
        enum ErrorCode
        {
            Success,
            Fail
        };

        using Result = container::Result<ErrorCode, ErrorCode::Success>;

        struct Frame
        {
            Ptr<std::vector<ReferenceLine>> references;
            Ptr<type::Scene> scene;
            Ptr<type::Vehicle> vehicle;
        };

    public:
        virtual ~PlannerBase() = default;

        Result Plan(const type::Trajectory & stitch_trajectory, Frame frame, type::Trajectory & result);

        virtual Result PlanOnReferenceLine(const type::TrajectoryPoint & init_point, ReferenceLine & referenceLine, Frame frame, Ptr<type::Trajectory> & result) = 0;
    };
}