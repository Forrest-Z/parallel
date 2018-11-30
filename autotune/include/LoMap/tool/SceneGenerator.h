/**
 * @file SceneGenerator.h
 */

#pragma once

#include <nox>
#include <LoMap/provider/MapProvider.h>
#include <LoMap/provider/GuideLineProvider.h>
#include <LoMap/provider/VehicleStateProvider.h>
#include <LoMap/provider/ObstacleProvider.h>
#include <LoMap/provider/TrafficLightProvider.h>
#include <LoMap/provider/StopLineProvider.h>
#include <LoMap/provider/SceneProvider.h>

namespace nox::app
{
    class SceneGenerator
        : public system::LoopThread
    {
    public:
        __asynchronous_thread
        nox_msgs::Scene GetMsg() const;

        __asynchronous_thread
        void UpdateMap(const std::string & source);

        __asynchronous_thread
        void UpdateVehicleState(const type::Odometry & state);

        __asynchronous_thread
        void UpdateObstacles(const nox_msgs::ObstacleArray & obstacles, bool is_global);

        __asynchronous_thread
        void UpdateTrafficLight(const type::Signal & signal);

    protected:
        void OnStart() override;

        void OnRun() override;

    private:
        MutexLockable(type::Scene) _ready_scene;
        MapProvider _map_provider;
        GuideLineProvider _guide_line_provider;
        VehicleStateProvider _vehicle_state_provider;
        ObstacleProvider _obstacle_provider;
        TrafficLightProvider _traffic_light_provider;
        StopLineProvider _stop_line_provider;
        SceneProvider _scene_provider;
    };
}