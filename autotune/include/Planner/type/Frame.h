/**
 * @brief Frame类，存储了一帧规划中的所有信息
 * @details
 * 1. 场景信息
 * 2. 车信息
 * 3. 规划结果
 */
#pragma once

#include <nox>

namespace nox::app
{
    class Frame
    {
    public:
        Frame();

    public:
        size_t ID() const;

        string Brief() const;

    public:
        type::Vehicle vehicle;
        type::Scene scene;
        type::Trajectory trajectory;

        size_t _ID;
    };
}