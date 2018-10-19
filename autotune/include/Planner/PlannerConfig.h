/**
 * @brief Planner模块使用的全局配置，可以在此引入头文件、全局缓存等。
 * （不建议手动定义全局变量）
 */

#pragma once

#include <nox>

namespace nox::app
{
    caching(type::Vehicle, EgoVehicle);
}