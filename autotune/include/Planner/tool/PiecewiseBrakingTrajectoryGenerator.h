/**
 * @file PiecewiseBrakingTrajectoryGenerator.h
 */

#pragma once

#include <nox>

namespace nox::app
{
    class PiecewiseBrakingTrajectoryGenerator
    {
    public:
        PiecewiseBrakingTrajectoryGenerator() = delete;

        /**
         * 尝试在给定距离处刹车到目标速度，根据距离进行加速度的调整
         * @param a_comfort 舒适的加速度（正值）
         * @param d_comfort 舒适的减速度（负值）
         * @param max_time 给定的时间约束
         * @return
         */
        static Ptr<math::Parametric<1>> Generate(
            double s_target, double s_curr,
            double v_target, double v_curr,
            double a_comfort, double d_comfort,
            double max_time);

        /**
         * 计算以给定的加速度减速的轨迹
         * @param v_target 目标速度，要求比v_curr小（刹车嘛）
         * @param dec 减速度（负值）
         * @return
         */
        static Ptr<math::Parametric<1>> Generate(double s_curr, double v_curr, double v_target, double dec);

        static double ComputeStopDistance(double v, double dec);

        static double ComputeStopDeceleration(double dist, double v);
    };
}