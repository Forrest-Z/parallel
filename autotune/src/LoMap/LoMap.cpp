#include <LoMap/LoMap.h>
#include <LoMap/.LoMapModule.h>

using namespace nox::app;
USING_NAMESPACE_NOX;

void LoMap::Initialize()
{
    Logger::GlobalLogLevel(Logger::Debug);
    Logger::DisplayTimeStamp(true);

    cache::WriteDeviceParameter(params.Device);
    Logger::D("LoMap") << string(params.Device);

    _scene_server.Subscribe({"scene"});
    _scene_server.AddRequestCallback(&LoMap::ProcessOnPlannerRequest, this);
    _scene_generator.Start();
}

void LoMap::Terminate()
{
    _scene_server.UnSubscribe();
    _scene_generator.Finish();
}

bool LoMap::ProcessOnvehicle_state(nav_msgs::Odometry vehicle_state)
{
    type::Odometry state;
    state.From(vehicle_state);
    _scene_generator.UpdateVehicleState(state);
    return true;
}

bool LoMap::ProcessOnobstacles(nox_msgs::ObstacleArray obstacles)
{
    _scene_generator.UpdateObstacles(obstacles, false);
    return true;
}

bool LoMap::ProcessOnold_map(nox_msgs::Road old_map)
{
    // TODO
    return true;
}

bool LoMap::ProcessOnPlannerRequest(
    const nox_msgs::GetScene::Request &request,
    const mailbox::Address &address,
    nox_msgs::GetScene::Response &response)
{
    response.scene = _scene_generator.GetMsg();
    return true;
}

bool LoMap::ProcessOnhdmap(std_msgs::String hdmap)
{
    _scene_generator.UpdateMap(hdmap.data);
    return true;
}

bool LoMap::ProcessOnvirtual_obstacles(nox_msgs::ObstacleArray virtual_obstacles)
{
    _scene_generator.UpdateObstacles(virtual_obstacles, true);
    return true;
}

bool LoMap::ProcessOntraffic_lights(traffic_light::msg_traffic_light_list lights)
{
    type::Signal signal;
    for(auto & i : lights.lights)
    {
        if(i.color == 2)
        {
            if(i.left)
                signal.direction |= scene::Leftward;
            if(i.right)
                signal.direction |= scene::Rightward;
            if(i.forward)
                signal.direction |= scene::Forward;
        }
    }

    _scene_generator.UpdateTrafficLight(signal);
    return true;
}


