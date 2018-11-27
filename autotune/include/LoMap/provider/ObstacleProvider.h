//
// Created by yarten on 18-11-26.
//

#pragma once

#include <nox>
#include <LoMap/type/Material.h>
#include <LoMap/type/MD5.h>

namespace nox::app
{
    class ObstacleProvider
    {
    public:
        void Initialize();

        __asynchronous_thread
        void Update(const std::vector<type::Obstacle> & obstacles, bool is_global);

        MD5<std::vector<type::Obstacle>> Produce(const MD5<type::Odometry> & state);

    private:
        void _Update(const std::vector<type::Obstacle> & src, Material<MD5<std::vector<type::Obstacle>>> & dst);

        void Process();

    private:
        Material<MD5<type::Odometry>> _state;
        Material<MD5<std::vector<type::Obstacle>>> _input_local;
        Material<MD5<std::vector<type::Obstacle>>> _input_global;

        MD5<std::vector<type::Obstacle>> _output;

        struct
        {
            type::Pose _lidar;
        } _device;
    };
}