/**
 * @brief Planner模块使用的全局配置，可以在此引入头文件、全局缓存等。
 * （不建议手动定义全局变量）
 */

#pragma once

#include <nox>
#include "../../../.param/template/Parameter.h"
#include "../../../.plugin/Plugin.h"

namespace nox::app
{
    caching(type::Vehicle, EgoVehicle);

    namespace key /// 用于模块间唯一标识某些信息的id集合
    {
#define DeclKey(_name_) const type::scene::ID _name_ = __COUNTER__ + 1

        const type::scene::ID Unknown = 0;
        DeclKey(TrafficLight);
        DeclKey(DeadEnd);
        DeclKey(Junction);
        DeclKey(MapEdge);
        DeclKey(RoadRule);
    }
}