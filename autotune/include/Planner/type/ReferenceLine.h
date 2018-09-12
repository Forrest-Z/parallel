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
        interface<type::Path> path;
        type::Scene::ID laneID;

    public:
        enum Priority
        {
            _0 = 0, _1, _2, _3
        };

        ReferenceLine();

    public: /// 操作接口

        void AddCost(Priority priority, double cost);

    public: /// 配置接口

        void SetDrivable(bool drivable);

        void SetStopPoint(double s);

    public: /// 查询接口

        bool PriorThan(const ReferenceLine & other) const;

        bool IsReachedEnd(Ptr<type::Vehicle> vehicle) const;

    private:
        std::array<double, 4> _priority;
        bool _drivable;
        double _stopPoint; // 使用路程来表示
    };
}