#include <LoMap/tool/SceneMaintainer.h>
#include <LoMap/tool/StaticSceneUpdater.h>

using namespace nox::app;


SceneMaintainer::SceneMaintainer()
    : _ready_scene(New<Scene>()),
      _temp_scene(New<Scene>()),
      _static_updater(_temp_scene),
      _dynamic_updater(_temp_scene),
      _overlap_builder(_temp_scene)
{

}

nox_msgs::Scene SceneMaintainer::ToMsg() const
{
    Locking(_visit_lock)
    {
        nox_msgs::Scene msg;
        _ready_scene->To(msg);
        return msg;
    }
}


void SceneMaintainer::UpdateState(const Odometry & state)
{
    Locking(_update_lock)
    {
        _dynamic_updater.Update(state);
    }
    UpdateScene();
}

void SceneMaintainer::UpdateMap(const nox_msgs::Road &source)
{
    Locking(_update_lock)
    {
        _static_updater.Update(source);
        _overlap_builder.Rebuild();
    }
    UpdateScene();
}

void SceneMaintainer::UpdateObstacles(const nox_msgs::ObstacleArray &obstacles, bool is_global)
{
    Locking(_update_lock)
    {
        _dynamic_updater.Update(obstacles, is_global);
        _overlap_builder.Rebuild();
    }
    UpdateScene();
}

void SceneMaintainer::UpdateMap(const std_msgs::String &source)
{
    Locking(_update_lock)
    {
        _static_updater.Update(source);
        _overlap_builder.Rebuild();
    }
    UpdateScene();
}

void SceneMaintainer::UpdateTrafficLight(const traffic_light::msg_traffic_light_list & lights)
{
    Locking(_update_lock)
    {
        _static_updater.Update(lights);
    }
    UpdateScene();
}

void SceneMaintainer::UpdateScene()
{
    Locking(_visit_lock)
        *_ready_scene = *_temp_scene;
}
