//
// Created by yarten on 18-11-27.
//

#pragma once

#include <nox>
#include <LoMap/type/Material.h>
#include <LoMap/type/MD5.h>

namespace nox::app
{
    class SceneProvider
    {
    public:
        void Produce(
            const MD5<vector<Ptr<type::GuideLine>>> & guideLines,
            const MD5<std::vector<type::Obstacle>>  & obstacles,
            const MD5<type::Odometry>               & state,
            type::Scene                             & scene
        );

    private:
        Material<MD5<vector<Ptr<type::GuideLine>>>> _guide_lines;
        Material<MD5<std::vector<type::Obstacle>>>  _obstacles;
        Material<MD5<type::Odometry>>               _vehicle_state;
    };
}