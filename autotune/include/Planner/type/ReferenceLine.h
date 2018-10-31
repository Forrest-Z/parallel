/**
 * @brief ReferenceLine参考线类，描述的是带有优先级、停止点等信息的轨迹
 */
#pragma once

#include <nox>
#include <array>

namespace nox::app
{
    class ReferenceLine : public type::GuideLine
    {
    public:
        enum Priority
        {
            _0 = 0, _1, _2, _3,
            Definite = _0,
            Important = _1,
            Available = _2,
            Insignificant = _3
        };

        ReferenceLine() = default;

        ReferenceLine(const type::GuideLine & guideLine);

    public: /// 操作接口
        void AddCost(Priority priority, double cost);

    public: /// 查询接口
        bool IsPriorThan(const ReferenceLine &other) const;

        bool IsReachedEnd(Ptr<type::Vehicle> vehicle) const;

        double Length() const;

        double CruisingSpeed() const;

        double StopPoint() const;

    public: /// 工具接口
        math::Frenet CalculateFrenet(Ptr<type::Vehicle> vehicle) const;

        math::Frenet CalculateFrenet(double x, double y, double theta) const;

    private:
        std::array<double, 4> _priority{1, 1, 1, 1};
    };
}