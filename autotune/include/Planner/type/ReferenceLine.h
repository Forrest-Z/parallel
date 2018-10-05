/**
 * @brief ReferenceLine参考线类，描述的是带有优先级、停止点等信息的轨迹
 */
#pragma once

#include <nox>
#include <array>

namespace nox::app
{
    class ReferenceLine
    {
    public:
        Ptr<const type::Path> path;
        scene::ID laneID;
        double width = 3.5; // 路宽，应该来自地图

    public:
        enum Priority
        {
            _0 = 0, _1, _2, _3
        };

        struct Target
        {
            double s; // 使用路程来表示
            double v; // 目标速度

            bool IsStop() const;
        };

        ReferenceLine();

        explicit ReferenceLine(const type::Lane & lane);

    public: /// 操作接口

        void AddCost(Priority priority, double cost);

    public: /// 配置接口

        void SetDrivable(bool drivable);

        void SetStopPoint(double s);

    public: /// 信息接口
        Target GetTarget() const;

    public: /// 查询接口

        bool IsPriorThan(const ReferenceLine &other) const;

        bool IsReachedEnd(Ptr<type::Vehicle> vehicle) const;

    public: /// 工具接口
        math::Frenet CalculateFrenet(double x, double y, double theta) const;

    private:
        std::array<double, 4> _priority;
        bool _drivable;
        Target _target;


    };
}