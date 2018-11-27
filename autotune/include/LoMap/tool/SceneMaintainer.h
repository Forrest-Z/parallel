/**
 * @brief 维护场景数据，包括读写同步、场景解析、数据关联等
 */
#pragma once

#include <nox>
#include "StaticSceneUpdater.h"
#include "DynamicSceneUpdater.h"
#include "SceneOverlapBuilder.h"

namespace nox::app
{
    class SceneMaintainer
        : public system::MutexLock
    {
    public:
        explicit SceneMaintainer();

        nox_msgs::Scene ToMsg() const;

    public:
        __asynchronous_thread
        void UpdateMap(const nox_msgs::Road & source);

        __asynchronous_thread
        void UpdateObstacles(const nox_msgs::ObstacleArray & obstacles, bool is_global);

        __asynchronous_thread
        void UpdateState(const type::Odometry & state);

        __asynchronous_thread
        void UpdateMap(const std_msgs::String & source);

        __asynchronous_thread
        void UpdateTrafficLight(const traffic_light::msg_traffic_light_list & lights);

    private:
        void UpdateStaticScene();

        void UpdateDynamicScene();

        void RefreshScene();

    private:
        Ptr<type::Scene> _ready_scene;
        Ptr<type::Scene> _temp_scene;
        system::MutexLock _visit_lock;
        system::MutexLock _static_update_lock;
        system::MutexLock _dynamic_update_lock;

        StaticSceneUpdater _static_updater;
        DynamicSceneUpdater _dynamic_updater;
        SceneOverlapBuilder _overlap_builder;
    };
}