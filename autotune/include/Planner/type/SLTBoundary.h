/**
 * @brief 在轨迹上建立的SL坐标系
 */
#pragma once

#include <nox>

namespace nox::app
{
    class SLTBoundary
    {
    public:
        type::Range s, l, t;
    };
}
