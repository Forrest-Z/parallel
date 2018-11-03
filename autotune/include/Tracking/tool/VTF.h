/**
 * @brief VTF横向控制算法
 */

#pragma once

#include "LateralController.h"

namespace nox::app
{
    class VTF
        : public LateralController
    {
    public:
        double Calculate(const type::Trajectory &path, const type::Vehicle &vehicle) override;

    };
}