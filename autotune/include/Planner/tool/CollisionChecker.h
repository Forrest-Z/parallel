/**
 * @brief 障碍物碰撞检测器
 * @details
 * 1. 假设最终检查的轨迹的采样间隔固定，并与该检测器建立时使用的参数一致
 * 2. 本类会根据相关规划参数，预计算用于加速检索
 */
#pragma once

#include <nox>
#include <Planner/type/ReferenceLine.h>
#include <vector>

namespace nox::app
{
    class CollisionChecker
    {
    public:
        CollisionChecker(Ptr<type::Scene> scene, double s, double l, Ptr<ReferenceLine> reference);

        CollisionChecker(Ptr<type::Scene> scene, double s, double l, Ptr<ReferenceLine> reference, double temporal_length);

        bool InCollision(const type::Trajectory & trajectory) const;

    private:
        void BuildPredictedEnvironment(Ptr<type::Scene> scene, double s, double l, Ptr<ReferenceLine> reference);

        /**
         * 忽略所有跟车同一个车道的后方障碍物
         * @param obstacle 障碍物
         * @param s 车所在的s，障碍物的s小于车的s，说明障碍物在车后方（相对于当前参考线而言）
         * @param l 车所在的l，当l小于路宽一半时，认为车在当前参考线上
         * @param referenceLine 参考线
         * @return 障碍物是否跟车同一车道线
         */
        bool ShouldIgnore(Ptr<type::Obstacle> obstacle, double s, double l, Ptr<ReferenceLine> referenceLine) const;

        math::Frenet GetFrenetCoordinate(const Pose & pose, Ptr<ReferenceLine> referenceLine) const;

    private:
        struct
        {
            double _time_resolution = 0.1;
            double _planning_temporal_length = 8.0;

            struct /// 碰撞检测缓冲边界（需要跟速度挂钩？）
            {
                double _lon = 2.0;
                double _lat = 0.5;
            } _buffer;
        } _param;

        /// 第一维度为时间，第二维度为各个障碍物
        std::vector<std::vector<type::Box>> _predicted_bouding_boxes;
    };
}