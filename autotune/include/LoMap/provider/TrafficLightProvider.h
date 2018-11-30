/**
 * @file TrafficLightProvider.h
 */

#pragma once

#include <nox>
#include <LoMap/type/Material.h>
#include <LoMap/type/MD5.h>

namespace nox::app
{
    class TrafficLightProvider
    {
    public:
        __asynchronous_thread
        void Update(const type::Signal & traffic_light);

        MD5<vector<Ptr<GuideLine>>> Produce(const MD5<type::Map> & map, const MD5<vector<Ptr<type::GuideLine>>> & guideLines);

    private:
        bool Process();

    private:
        Material<type::Signal> _signal;
        Material<MD5<vector<Ptr<type::GuideLine>>>> _guide_lines;
        Material<MD5<type::Map>> _map;

        MD5<vector<Ptr<type::GuideLine>>> _output;
    };
}