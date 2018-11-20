/**
 * @file Clipping.h
 */
#pragma once

#include <Tracking/tool/Filter.h>

namespace nox::app::rule
{
    class Clipping
        : public Filter::Rule
    {
    public:
        Clipping() = default;

        /**
         * 使用限幅和初始值构造该规则
         * @param limit 一个正值，同时代表增幅和减幅限制（负值认为不限制）
         * @param init_data 初始化值
         */
        Clipping(double limit, double init_data = 0);

        Clipping(double increase_limt, double decrease_limit, double init_data);

        double Apply(double raw) override;

    private:
        double _last_data = 0;
        double _increase_limit = -1;
        double _decrease_limit = -1;
    };
}