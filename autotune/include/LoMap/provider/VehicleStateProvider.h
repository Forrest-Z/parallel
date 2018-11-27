//
// Created by yarten on 18-11-26.
//

#pragma once

#include <nox>
#include <LoMap/tool/PoseEstimator.h>
#include <LoMap/type/Material.h>
#include <LoMap/type/MD5.h>

namespace nox::app
{
    class VehicleStateProvider
        : public system::MutexLock
    {
    public:
        __asynchronous_thread
        void Update(const Odometry & state);

        MD5<Odometry> Produce();

        MD5<Odometry> Produce(Time time);

    private:
        Material<Odometry> _input;
        MD5<Odometry> _output;
    };
}