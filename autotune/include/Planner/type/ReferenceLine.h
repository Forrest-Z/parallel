/**
 * @brief ReferenceLine参考线类，描述的是带有优先级、停止点等信息的轨迹
 */
#pragma once

#include <nox>
#include <array>

using namespace nox::type;
using nox::container::Result;

namespace nox::app
{
    class ReferenceLine :
        public GuideLine
    {
    public: /// 构造器
        enum Priority
        {
            _0 = 0, _1, _2, _3,
            Definite = _0,
            Important = _1,
            Available = _2,
            Insignificant = _3
        };

        ReferenceLine() = default;

        ReferenceLine(const ReferenceLine & other) = default;

        ReferenceLine(const GuideLine & guideLine);

        ReferenceLine &operator=(const ReferenceLine & other) = default;

        ReferenceLine &operator=(const GuideLine & guideLine);


    public: /// 操作接口
        void AddCost(Priority priority, double cost);

        /**
         * 杀死该条参考线，不会被规划
         */
        void Kill();

    public: /// 信息接口
        double Length() const;

        /**
         * 根据位置信息，得到那一点引导线的速度
         * @param s 引导线s距离
         * @return s距离处的速度（无速度信息，则返回默认速度）
         */
        double CruisingSpeed(double s = -1) const;

        Bound GetBoundary(double s) const;

    public: /// 查询接口
        bool IsPriorThan(const ReferenceLine &other) const;

        bool IsReachedEnd(const Vehicle & vehicle) const;

        bool IsDead() const;

        /**
         * 判断位置是否在停止线上
         * @param position 欲判断的位置
         * @param th 范围阈值
         * @return 是否压着停止线
         */
        bool IsOverStopLine(double s, double th) const;

        Result<bool> IsNormal(const Trajectory &trajectory, const Vehicle &vehicle) const;


    public: /// 工具接口
        math::Frenet CalculateFrenet(const Vehicle & vehicle) const;

        math::Frenet CalculateFrenet(const Pose & pose) const;

        math::Frenet CalculateFrenet(double x, double y, double theta) const;

        double GetNextStopLine(double s) const;

    private: /// 内部处理函数
        void Setup();

    private:
        std::array<double, 4>       _priority{1, 1, 1, 1};      // 引导线优先级{绝对的，重要的，普通的，无关痛痒的}
        bool                        _killed = false;            // 是否不可使用
        std::vector<StopLine>       _stop_lines;                // 有序的引导线信息
        container::Segment<Bound>   _speed_controls;            // 速度控制段信息
        container::Segment<Ptr<math::Parametric<1>>>   _boundaries;  // 边界段信息
    };
}