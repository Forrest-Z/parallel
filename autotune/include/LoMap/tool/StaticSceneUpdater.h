/**
 * @brief 地图数据解析器，作为一个中间转换工具之用
 * @details
 * 1. 仅处理与地图数据相关的东西；
 * 2. 不会维护无关地图的数据联系（如障碍物）
 */
#pragma once

#include <nox>
#include <old_nox_msgs.h>
#include "SceneObjectCreator.h"
using namespace nox::type;

namespace nox::app
{
    class StaticSceneUpdater
    {
    public:
        explicit StaticSceneUpdater(Ptr<Scene> scene);

        void Update(const nox_msgs::Road &source);

        struct
        {
            double default_lane_width = 3.5;
        } param;

    private:
        void ClearSceneObjects();


    private:
        Ptr<Scene> _scene;

        SceneObjectCreator<Lane> _lane_creator;
        SceneObjectCreator<scene::Group> _lane_segment_creator;
        SceneObjectCreator<Junction> _junction_creator;
    };


}