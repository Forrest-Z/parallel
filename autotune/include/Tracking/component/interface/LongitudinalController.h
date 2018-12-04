/**
 * @brief 纵向控制器基类
 */

#pragma once

#include <nox>

namespace nox::app
{
    class LongitudinalController
    {
    public:
        virtual ~LongitudinalController() = default;

        virtual double Calculate(const type::Trajectory & path, const type::Vehicle & vehicle) = 0;
    };
}