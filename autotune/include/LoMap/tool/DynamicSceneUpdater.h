/**
 * @brief 场景中动态物的更新器，如障碍物、红绿灯等，以及场景本身的位姿。
 */
#pragma once

#include "PoseEstimator.h"
#include "SceneObjectCreator.h"
#include <LoMap/LoMapConfig.h>

namespace nox::app
{
    class DynamicSceneUpdater
    {
    public:
        explicit DynamicSceneUpdater(Ptr<type::Scene> scene);

        void Update(const Odometry & state);

        void Update(const nox_msgs::ObstacleArray & obstacles, bool is_global = false);

    private:
        PoseEstimator _pose_estimator;
        Ptr<type::Scene> _scene;

        SceneObjectCreator<type::Obstacle> _obstacle_creator;

        struct
        {
            type::Pose _lidar;
        } _device;
    };
}