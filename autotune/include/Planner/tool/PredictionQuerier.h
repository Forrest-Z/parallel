/**
 * @brief 障碍物在参考线的运动的预测工具
 */
#pragma once

#include <nox>
#include <Planner/type/ReferenceLine.h>

namespace nox::app
{
    class PredictionQuerier
    {
    public:
        PredictionQuerier(Ptr<type::Scene> scene, Ptr<ReferenceLine> reference);

        /**
         * 将目标时间点障碍物所处的速度状态，在参考线的目标距离点上作投影
         * @param obstacle_id 想要查询的障碍物ID
         * @param s 想要查询的参考线的位置
         * @param t 障碍物运动轨迹的某个时间点（所有障碍物时间起点齐平）
         * @return 投影的速度
         */
        double ProjectVelocityAlongReferenceLine(
            const scene::ID & obstacle_id,
            double s,
            double t
        );

    private:
        Ptr<type::Scene> _scene;
        Ptr<ReferenceLine> _reference;
    };
}