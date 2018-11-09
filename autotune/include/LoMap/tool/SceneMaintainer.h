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
        explicit SceneMaintainer(Ptr<type::Scene> scene);

        nox_msgs::Scene ToMsg() const;

    public:
        void UpdateMap(const nox_msgs::Road & source);

        void UpdateObstacles(const nox_msgs::ObstacleArray & obstacles, bool is_global);

        void UpdateState(const type::Odometry & state);

        void UpdateMap(const std_msgs::String & source);

    private:
        Ptr<type::Scene> _scene;
        StaticSceneUpdater _static_updater;
        DynamicSceneUpdater _dynamic_updater;
        SceneOverlapBuilder _overlap_builder;
    };
}