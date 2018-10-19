/**
 * @brief 包含了lattice的一些通用定义
 */
#pragma once

#include <nox>
#include <vector>

namespace nox::app::lattice
{
    using Bundle = std::vector<Ptr<math::Parametric<1>>>;
    struct State : public math::Derivative<3>
    {
        double t;

        State();

        State(const math::Derivative<3> & state, double t);

        State(double s0, double s1, double s2, double t);
    };

    class Curve : public math::Parametric<1>
    {
    public:
        double target_time = 0;
        double target_speed = 0;
        double target_position = 0;

    public:
        explicit Curve(Ptr<math::Parametric<1>> curve);

        double Calculate(size_t order, double param) const override;

        double Boundary() const override;

    private:
        Ptr<math::Parametric<1>> _curve;
    };

    struct Combination
    {
        Ptr<math::Parametric<1>> lon;
        Ptr<math::Parametric<1>> lat;
        double cost_sum = 0;       // 根据比例加权和的权重
        std::vector<double> costs; // 各种各样的cost

        Combination() = default;

        Combination(Ptr<math::Parametric<1>> lon, Ptr<math::Parametric<1>> lat);

        bool operator<(const Combination & other) const;
    };


}
