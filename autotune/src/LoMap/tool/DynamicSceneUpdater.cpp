#include <LoMap/tool/DynamicSceneUpdater.h>
USING_NAMESPACE_NOX;
using namespace nox::app;

DynamicSceneUpdater::DynamicSceneUpdater(Ptr<type::Scene> scene)
    : _scene(scene)
{
    assert(scene);
}

void DynamicSceneUpdater::Update(const Odometry &state)
{
    _pose_estimator.Update(state);
}

void DynamicSceneUpdater::Update(const nox_msgs::ObstacleArray &obstacles)
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
        obstacle->operator*=(current_pose);

        _scene->Obstacles[obstacle->id] = obstacle;
    }
}

