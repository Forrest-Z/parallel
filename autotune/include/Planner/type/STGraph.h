/**
 * @brief 轨迹-时间图
 */
#pragma once

#include <nox>
#include <Planner/type/ReferenceLine.h>
#include <Planner/type/SLTBoundary.h>

namespace nox::app
{
    class STPoint
    {
    public:
        STPoint() = default;

        STPoint(double t, type::Range s);

        STPoint(double t, double s_begin, double s_end);

    public:
        double      t = 0;
        type::Range s;
    };

    /**
     * @class STObstacle
     * 表示ST障碍物的方式如下图所示：
     * (t, s_range)，即某时刻障碍物占用的s范围
     * S
     * ^
     * |  Se0  ..... Se1
     * |  Sb0  ..... Sb1
     * + -------------------> T
     *     t0        t1
     */
    class STObstacle
    {
    public:
        std::vector<STPoint> trajectory;

    public:
        type::Range EstimateAtTime(double t) const;
    };

    class STGraph
    {
    public:
        STGraph(
            Ptr<type::Scene> scene,
            const ReferenceLine & referenceLine,
            double start_s, double end_s,
            double start_t, double end_t,
            double path_width,
            double time_resolution);

        void SetupObstacles(
            Ptr<type::Scene> scene,
            const ReferenceLine & referenceLine);

        void AddDynamicObstacle(
            const type::Obstacle &obstacle,
            const ReferenceLine &referenceLine
        );

        void AddStaticObstacle(
            const type::Obstacle &obstacle,
            const ReferenceLine &referenceLine);

    public:
        Ptr<STObstacle> GetObstacle(scene::ID id);

        double TimeResolution() const;

        const type::Range & SRange() const;

        const type::Range & TRange() const;

        std::vector<type::Bound> GetLateralBounds(double s_start, double s_end, double s_resolution);

    private:
        SLTBoundary ComputeObstacleBoundary(
            const type::Box & box,
            const ReferenceLine & referenceLine
        ) const;

    private:
        type::Range _s, _t;
        double _half_path_width;
        double _time_resolution;

        std::unordered_map<scene::ID, Ptr<STObstacle>> _obstacles;

    public:
        std::unordered_map<scene::ID, Ptr<STObstacle>> & Obstacles()
        {
            return _obstacles;
        }
    };
}