/**
 * @brief 根据车辆的状态，以及时间戳的变换，预估当前位置
 */
#pragma once

#include <nox>
using nox::type::Pose;
using nox::type::Odometry;
using nox::type::Stamped;

namespace nox::app
{
    class PoseEstimator
    {
    public:
        PoseEstimator();

        void Update(const Odometry & stamped_odometry);

        Pose Estimate(const type::Time & current_time);

    private:
        container::History<Odometry> _history;
    };
}