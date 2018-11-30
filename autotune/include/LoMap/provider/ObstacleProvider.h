//
// Created by yarten on 18-11-26.
//

#pragma once

#include <nox>
#include <LoMap/type/Material.h>
#include <LoMap/type/MD5.h>
#include <LoMap/provider/VehicleStateProvider.h>

namespace nox::app
{
    class ObstacleProvider
    {
    public:
        void Initialize();

        __asynchronous_thread
        void Update(const nox_msgs::ObstacleArray & obstacles, bool is_global);

        MD5<std::vector<type::Obstacle>> Produce(VehicleStateProvider & state_provider);

    private:
        void _Update(const nox_msgs::ObstacleArray & src, Material<MD5<nox_msgs::ObstacleArray>> & dst);

        void Process(VehicleStateProvider & state_provider);

    private:
        Material<MD5<nox_msgs::ObstacleArray>> _input_local;
        Material<MD5<nox_msgs::ObstacleArray>> _input_global;

        MD5<std::vector<type::Obstacle>> _output;

        struct
        {
            type::Pose _lidar;
        } _device;
    };
}