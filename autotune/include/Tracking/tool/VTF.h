/**
 * @brief VTF横向控制算法
 */

#pragma once

#include "LateralController.h"
#include <Tracking/tool/Filter.h>

namespace nox::app
{
    class VTF
        : public LateralController
    {
    public:
        VTF();

        double Calculate(const type::Trajectory &path, const type::Vehicle &vehicle) override;

    private:
        math::filter::RC_1o_1d _da_filter;
        Filter _a_error_filter;
    };
}