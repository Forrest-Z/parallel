/**
 * @brief 简单的纵向控制器，直接听从planner的计算结果
 */

#pragma once

#include "LongitudinalController.h"

namespace nox::app
{
    class SimpleLongitudinalController
        : public LongitudinalController
    {
    public:
        double Calculate(const type::Trajectory &path, const type::Vehicle &vehicle) override;
    };
}