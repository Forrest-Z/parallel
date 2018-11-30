#include <LoMap/LoMap.h>
#include <LoMap/.LoMapModule.h>

using namespace nox::app;
USING_NAMESPACE_NOX;

#define USE_NEW_METHOD

void LoMap::Initialize()
{
    Logger::GlobalLogLevel(Logger::Debug);
    Logger::DisplayTimeStamp(true);

    cache::WriteDeviceParameter(params.Device);
    Logger::D("LoMap") << string(params.Device);

    _scene_maintainer = New<SceneMaintainer>();
    _scene_server.Subscribe({"scene"});
    _scene_server.AddRequestCallback(&LoMap::ProcessOnPlannerRequest, this);

#ifdef USE_NEW_METHOD
    _scene_generator.Start();
#endif
}

bool LoMap::ProcessOnvehicle_state(nav_msgs::Odometry vehicle_state)
{
    type::Odometry state;
    state.From(vehicle_state);

#ifdef USE_NEW_METHOD
    _scene_generator.UpdateVehicleState(state);
#else
    _scene_maintainer->UpdateState(state);
#endif
    return true;
}

bool LoMap::ProcessOnobstacles(nox_msgs::ObstacleArray obstacles)
{
#ifdef USE_NEW_METHOD
    _scene_generator.UpdateObstacles(obstacles, false);
#else
    _scene_maintainer->UpdateObstacles(obstacles, false);
#endif
    return true;
}

bool LoMap::ProcessOnold_map(nox_msgs::Road old_map)
{
    _scene_maintainer->UpdateMap(old_map);
    return true;
}

bool LoMap::ProcessOnPlannerRequest(
    const nox_msgs::GetScene::Request &request,
    const mailbox::Address &address,
    nox_msgs::GetScene::Response &response)
{
#ifdef USE_NEW_METHOD
    response.scene = _scene_generator.GetMsg();
#else
    response.scene = _scene_maintainer->ToMsg();
    response.scene.egoVehicle.odometry.pose.pose = request.location.pose;
#endif
    return true;
}

bool LoMap::ProcessOnhdmap(std_msgs::String hdmap)
{
#ifdef USE_NEW_METHOD
    _scene_generator.UpdateMap(hdmap.data);
#else
    if(mailboxes.hdmap.ReceiveHistoryCount() > 1)
    {
        auto last_hdmap = mailboxes.hdmap.LastReceive(1);
        if(last_hdmap.data == hdmap.data)
            return true;
    }

    _scene_maintainer->UpdateMap(hdmap);
#endif
    return true;
}

bool LoMap::ProcessOnvirtual_obstacles(nox_msgs::ObstacleArray virtual_obstacles)
{
#ifdef USE_NEW_METHOD
    _scene_generator.UpdateObstacles(virtual_obstacles, true);
#else
    _scene_maintainer->UpdateObstacles(virtual_obstacles, true);
#endif
    return true;
}

bool LoMap::ProcessOntraffic_lights(traffic_light::msg_traffic_light_list lights)
{
#ifdef USE_NEW_METHOD
    type::Signal signal;
    for(auto & i : lights.lights)
    {
        if(i.color == 2)
        {
            if(i.left)
                signal.direction = scene::Direction(signal.direction | scene::Leftward);
            if(i.right)
                signal.direction = scene::Direction(signal.direction | scene::Rightward);
            if(i.forward)
                signal.direction = scene::Direction(signal.direction | scene::Forward);
        }
    }

    _scene_generator.UpdateTrafficLight(signal);
#else
    _scene_maintainer->UpdateTrafficLight(lights);
#endif
    return true;
}

