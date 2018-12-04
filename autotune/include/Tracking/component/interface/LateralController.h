/**
 * @brief 侧向控制器
 */

#pragma once

#include <nox>

namespace nox::app
{
    class LateralController
    {
    public:
        virtual ~LateralController() = default;

        virtual double Calculate(const type::Trajectory & path, const type::Vehicle & vehicle) = 0;

        virtual void Initialize() {}
    };
}