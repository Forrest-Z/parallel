//
// Created by yarten on 18-11-27.
//

#pragma once

#include <nox>
#include <LoMap/type/MD5.h>

namespace nox::app
{
    class StopLineProvider
    {
    public:
        void Produce(MD5<vector<Ptr<nox::type::GuideLine>>> & guideLines);

        const static std::string TrafficLight;

        const static std::string DeadEnd;
    };
}