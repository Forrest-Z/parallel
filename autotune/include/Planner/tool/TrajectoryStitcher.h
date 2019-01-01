/**
 * @brief TrajectoryStitcher 用于从旧轨迹产生缝合新轨迹的一段轨迹
 */
#pragma once

#include <nox>
using nox::container::Result;
using namespace nox::type;

namespace nox::app
{
    class TrajectoryStitcher
    {
    public:
        /**
         * 产生初始化的缝合轨迹（根据当前车的位置、速度、转向，生成一小段位移轨迹）
         */
        Trajectory InitialTrajectory(const Vehicle & vehicle);

        /**
         * 产生短时间保持的缝合轨迹（根据当前车的位置）
         */
        Trajectory StitchTrajectoryByPosition(
            const Trajectory & last_trajectory,
            const Vehicle & vehicle,
            double stitch_time
        );

        Trajectory FromLastTrajectoryByPosition(
            const Trajectory & last_trajectory,
            const Vehicle & vehicle,
            OUT Result<bool> * status = nullptr
        );

        Result<bool> Check(const Trajectory & trajectory, const Vehicle & vehicle) const;

    private:
        Trajectory ComputeTrajectory(Position position, double theta, double kappa, double v, double a, double time_sum);

    public:
        struct
        {
            double initial_stitch_time = 0.0; // 秒，产生初始轨迹的时间长度
            double stitch_time = 1.0;         // 秒，产生缝合轨迹的时间长度
            double density = 1.0;             // 米，缝合轨迹步长

            struct
            {
                struct
                {
                    double lateral_offset = 5;      // 米，侧向容忍偏移上限
                    double longitudinal_offset = 5; // 米，纵向容忍偏移上限
                    double distance = 2;            // 米，欧拉距离容忍偏移上界
                    double too_short = 0.5;         // 米，已有轨迹过短重规划下界
                    double speed_diff = 3.0;        // m/s，最近匹配点速度差异上限
                } replan; /// 处理重规划的参数
            } threshold; /// 阈值对象
        } param; /// 参数对象

    private:
        double _planning_time = 0;
    };
}