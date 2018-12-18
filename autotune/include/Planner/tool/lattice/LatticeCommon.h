/**
 * @brief 包含了lattice的一些通用定义
 */
#pragma once

#include <nox>
#include <vector>
#include <Planner/type/ReferenceLine.h>

namespace nox::app::lattice
{
    struct CostFactor
    {
        double all            = 1.0;
        double s_travelled    = 1.0;
        double v_reached      = 1.0;
        double lateral_offset = 1.0;
    };

    struct State : public math::Derivative<2>
    {
        double t;

        CostFactor cost_factor;

        State();

        State(const math::Derivative<2> & state, double t, double priority_factor = 1.0);

        State(double s0, double s1, double s2, double t, double priority_factor = 1.0);
    };

    class Curve : public math::Parametric<1>
    {
    public:
        State state;

    public:
        explicit Curve(Ptr<math::Parametric<1>> curve);

        double Calculate(size_t order, double param) const override;

        double Upper() const override;

    private:
        Ptr<math::Parametric<1>> _curve;
    };

    using Bundle = std::vector<Ptr<Curve>>;

    struct Combination
    {
        Ptr<Curve> lon;
        Ptr<Curve> lat;
        double cost_sum = 0;       // 根据比例加权和的权重
        std::vector<double> costs; // 各种各样的cost

        Combination() = default;

        Combination(Ptr<Curve> lon, Ptr<Curve> lat);

        bool operator<(const Combination & other) const;
    };
}
