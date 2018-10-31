#include <LoMap/tool/SceneMaintainer.h>
#include <LoMap/tool/StaticSceneUpdater.h>

using namespace nox::app;


SceneMaintainer::SceneMaintainer(nox::Ptr<nox::type::Scene> scene)
    : _scene(scene), _static_updater(scene), _dynamic_updater(scene),
      _overlap_builder(scene)
{

}

nox_msgs::Scene SceneMaintainer::ToMsg() const
{
    Synchronized(this)
    {
        nox_msgs::Scene msg;
        _scene->To(msg);
        return msg;
    }
}


void SceneMaintainer::UpdateState(const Odometry & state)
{
    Synchronized(this)
        _dynamic_updater.Update(state);
}

void SceneMaintainer::UpdateMap(const nox_msgs::Road &source)
{
    Synchronized(this)
    {
        _static_updater.Update(source);
        _overlap_builder.Rebuild();
    }
}

void SceneMaintainer::UpdateObstacles(const nox_msgs::ObstacleArray &obstacles)
{
    Synchronized(this)
    {
        _dynamic_updater.Update(obstacles);
        _overlap_builder.Rebuild();
    }
}

void SceneMaintainer::UpdateMap(const std_msgs::String &source)
{
    Synchronized(this)
    {
        _static_updater.Update(source);
        _overlap_builder.Rebuild();
    }
}
