/**
 * @brief TrajectoryStitcher 用于从旧轨迹产生缝合新轨迹的一段轨迹
 */
#pragma once

#include <nox>

namespace nox::app
{
    class TrajectoryStitcher
    {
    public:
        /**
         * 产生初始化的缝合轨迹（根据当前车的位置、速度、转向，生成一小段位移轨迹）
         */
        type::Trajectory InitialTrajectory(const type::Vehicle & vehicle);

        type::Trajectory FromLastTrajectory(
            const type::Vehicle & vehicle,
            const type::Trajectory & last_trajectory,
            OUT bool * replan = nullptr);

        type::Trajectory FromLastTrajectoryByPosition(
            const type::Vehicle & vehicle,
            const type::Trajectory & last_trajectory,
            OUT bool * replan = nullptr
        );

    private:
        type::Trajectory ComputeTrajectory(type::Position position, double theta, double kappa, double v, double a, double time_sum);

    public:
        struct
        {
            double initial_stitch_time = 1.0; // 秒，产生初始轨迹的时间长度
            double stitch_time = 1.0;         // 秒，产生缝合轨迹的时间长度
            double density = 0.2;             // 米，缝合轨迹步长

            struct
            {
                struct
                {
                    double lateral_offset = 5;    // 米，侧向容忍偏移上限
                    double longitudinal_offset = 5; // 米，纵向容忍偏移上限
                } replan; /// 处理重规划的参数
            } threshold; /// 阈值对象
        } param; /// 参数对象

    private:
        double _planning_time = 0;
    };
}