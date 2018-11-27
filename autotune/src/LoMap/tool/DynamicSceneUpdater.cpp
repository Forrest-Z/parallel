#include <LoMap/tool/DynamicSceneUpdater.h>
#include <iostream>
USING_NAMESPACE_NOX;
using namespace nox::app;

DynamicSceneUpdater::DynamicSceneUpdater(Ptr<type::Scene> scene)
    : _scene(scene)
{
    assert(scene);

    auto & device_params = cache::ReadDeviceParameter();

    if(not device_params.Lidar.empty())
    {
        auto & lidar = device_params.Lidar[0];

        _device._lidar.Set(
            Position(lidar.x, lidar.y, lidar.z),
            Rotation(Degree(lidar.yaw), Degree(lidar.pitch), Degree(lidar.roll))
        );

        _device._lidar = _device._lidar.Inverse();

        Logger::D("DynamicSceneUpdater") << "Lidar pose: " << _device._lidar.ToString();
    }

}

void DynamicSceneUpdater::Update(const Odometry &state)
{
    _pose_estimator.Update(state);
}

void DynamicSceneUpdater::Update(const nox_msgs::ObstacleArray &obstacles, bool is_global)
{
    Time current_time;
    current_time.From(obstacles.header);
    Pose current_pose = _pose_estimator.Estimate(current_time);

    _scene->Obstacles.clear();

    for(auto & i : obstacles.obstacles)
    {
        auto obstacle = _obstacle_creator.Create();

        obstacle->From(i);

        /// 将障碍物姿态转换到全局坐标下
        if(not is_global)
        {
            obstacle->operator*=(current_pose * _device._lidar);
        }

        _scene->Obstacles[obstacle->id] = obstacle;
    }

    Logger::I("LoMap") << "Receive Obstacles: " << _scene->Obstacles.size();
}

