/**
 * @brief 局部规划基类
 */
#pragma once

#include <Planner/type/ReferenceLine.h>
#include <nox>
#include <vector>
using nox::container::Result;

namespace nox::app
{
    class PlannerBase
    {
    public:
        struct Frame
        {
            std::vector<Ptr<ReferenceLine>> references;
            Ptr<type::Scene> scene;
            Ptr<type::Vehicle> vehicle;
            Ptr<type::Trajectory> stitch;
        };

    public:
        virtual ~PlannerBase() = default;

        /**
         * 从当前帧规划到result
         * @param frame 当前帧规划信息（车状态、引导线、规划场景等）
         * @param result 规划结果
         * @return 规划状态
         */
        virtual Result<bool> Plan(const Frame & frame, type::Trajectory & result) = 0;

        /**
         * 检查frame.stitch轨迹在frame下的合理性
         * @param frame 当前规划信息，其中包含上一条规划的轨迹
         * @return 检查状态
         */
        virtual Result<bool> Check(const Frame & frame) = 0;
    };
}