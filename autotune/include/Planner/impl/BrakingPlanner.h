/**
 * @file BrakingPlanner.h，规划失败后的制动规划
 */

#pragma once

#include <Planner/PlannerBase.h>

namespace nox::app
{
    class BrakingPlanner :
        public PlannerBase
    {
    public:
        Result<bool> Plan(const Frame &frame, type::Trajectory &result) override;

        /**
         * @deprecated 没有用的
         */
        Result<bool> Check(const Frame &frame) override;
    };
}