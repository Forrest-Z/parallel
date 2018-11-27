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
    Locking(_dynamic_update_lock)
    {
        _dynamic_updater.Update(state);
        UpdateDynamicScene();
    }
}

void SceneMaintainer::UpdateMap(const nox_msgs::Road &source)
{
    Locking(_static_update_lock)
    {
        _static_updater.Update(source);
        UpdateStaticScene();
    }
}

void SceneMaintainer::UpdateObstacles(const nox_msgs::ObstacleArray &obstacles, bool is_global)
{
    Locking(_dynamic_update_lock)
    {
        _dynamic_updater.Update(obstacles, is_global);
        UpdateDynamicScene();
    }
}

void SceneMaintainer::UpdateMap(const std_msgs::String &source)
{
    Locking(_static_update_lock)
    {
        _static_updater.Update(source);
        UpdateStaticScene();
    }
}

void SceneMaintainer::UpdateTrafficLight(const traffic_light::msg_traffic_light_list & lights)
{
    Locking(_static_update_lock)
    {
        _static_updater.Update(lights);
    }
}

void SceneMaintainer::UpdateStaticScene()
{
    Locking(_visit_lock)
    {
        _ready_scene->GuideLines.clear();
        for(auto & i : _temp_scene->GuideLines)
        {
            _ready_scene->GuideLines[i.first] = New<GuideLine>(*i.second);
        }

        RefreshScene();
    }
}

void SceneMaintainer::UpdateDynamicScene()
{
    Locking(_visit_lock)
    {
        if(_temp_scene->EgoVehicle)
            _ready_scene->EgoVehicle = New<Vehicle>(*_temp_scene->EgoVehicle);

        _ready_scene->Obstacles.clear();
        for(auto & i : _temp_scene->Obstacles)
        {
            _ready_scene->Obstacles[i.first] = New<Obstacle>(*i.second);
        }

        RefreshScene();
    }
}

void SceneMaintainer::RefreshScene()
{
    _ready_scene->Refresh({"lomap", "test", "scene"});
}
