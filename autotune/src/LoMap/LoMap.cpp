#include <LoMap/LoMap.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

void LoMap::Initialize()
{
    _scene_maintainer = New<SceneMaintainer>();
    _scene_server.Subscribe({"scene"});
    _scene_server.AddRequestCallback(&LoMap::ProcessOnPlannerRequest, this);

    Logger::GlobalLogLevel(Logger::Debug);
}

bool LoMap::ProcessOnvehicle_state(nav_msgs::Odometry vehicle_state)
{
    type::Odometry state;
    state.From(vehicle_state);
    _scene_maintainer->UpdateState(state);
    return true;
}

bool LoMap::ProcessOnobstacles(nox_msgs::ObstacleArray obstacles)
{
    _scene_maintainer->UpdateObstacles(obstacles, false);
    return true;
}

bool LoMap::ProcessOnold_map(nox_msgs::Road old_map)
{
    Logger::I("LoMap") << "Receive new map !";
    _scene_maintainer->UpdateMap(old_map);
    return true;
}

bool LoMap::ProcessOnPlannerRequest(
    const nox_msgs::GetScene::Request &request,
    const mailbox::Address &address,
    nox_msgs::GetScene::Response &response)
{
    response.scene = _scene_maintainer->ToMsg();
    response.scene.egoVehicle.odometry.pose.pose = request.location.pose;
    return true;
}

bool LoMap::ProcessOnhdmap(std_msgs::String hdmap)
{
    _scene_maintainer->UpdateMap(hdmap);
    return true;
}

bool LoMap::ProcessOnvirtual_obstacles(nox_msgs::ObstacleArray virtual_obstacles)
{
    _scene_maintainer->UpdateObstacles(virtual_obstacles, true);
    return true;
}

bool LoMap::ProcessOntraffic_lights(traffic_light::msg_traffic_light_list traffic_lights)
{

    return true;
}

