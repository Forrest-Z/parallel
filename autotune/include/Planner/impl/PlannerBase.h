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
            Fail,
            InCollision
        };

        using Result = container::Result<ErrorCode, ErrorCode::Success>;

        struct Frame
        {
            std::vector<Ptr<ReferenceLine>> references;
            Ptr<type::Scene> scene;
            Ptr<type::Vehicle> vehicle;
        };

    public:
        virtual ~PlannerBase() = default;

        virtual Result Plan(const type::Trajectory & stitch_trajectory, Frame frame, type::Trajectory & result) = 0;

        virtual Result Check(const type::Trajectory & trajectory, const Frame & frame) = 0;

    public:
        static string ParseErrorCode(ErrorCode code);
    };
}