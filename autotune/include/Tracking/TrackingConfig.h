/**
 * @brief Tracking模块使用的全局配置，可以在此引入头文件、全局缓存等。
 * 1. 使用caching原语，定义全局缓存
 *
 * （不建议手动定义全局变量）
 */

#pragma once

#include <nox>
#include "../../../.param/template/Parameter.h"
#include "../../../.plugin/Plugin.h"

namespace nox::app
{
    caching(parameter::VehicleParameter, VehicleParameter);
    caching(parameter::VTFParameter, VTFParameter);
    caching(parameter::PCPIDParameter, PCPIDParameter);
}