/**
 * @brief 根据车体设定（最大加速度），以及当前的设置（v, t, s)，
 * 得到可行驶区域（关于t, v, s的区域）
 */
#pragma once

#include <nox>

namespace nox::app
{
    class FeasibleRegion
    {
    public:
        explicit FeasibleRegion(double s, double v, double a);

        double SUpper(double t) const;

        double SLower(double t) const;

        double VUpper(double t) const;

        double VLower(double t) const;

        double TLower(double s) const;

        double ComfortVUpper(double t) const;

        double ComfortVLower(double t) const;

    private:
        double _s, _v, _a;
        double _t_at_zero_speed;
        double _s_at_zero_speed;
        double _base_speed;
        
        double _comfort_t_at_zero_speed;
        double _comfort_s_at_zero_speed;

        type::Bound _longitudinal_acceleration;
        type::Bound _comfort_lon_acceleration;
    };
}