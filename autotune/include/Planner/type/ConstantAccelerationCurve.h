/**
 * @brief 描述恒定加速的的轨迹
 */
#pragma once

#include <nox>
#include <vector>

namespace nox::app
{
    class ConstantAccelerationCurve : public math::Parametric
    {
    public:
        ConstantAccelerationCurve(double start_s, double start_v);

        void PushSegment(double a, double dt);

        void PopSegment();

        double Calculate(size_t order, double param) const override;

        double Boundary() const override;

    private:
        double CalculateS(double t) const;

        double CalculateV(double t) const;

        double CalculateA(double t) const;

    private:
        std::vector<double> _s, _v, _t, _a;
    };
}
