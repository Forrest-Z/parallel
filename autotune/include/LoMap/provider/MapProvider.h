//
// Created by yarten on 18-11-26.
//

#pragma once

#include <LoMap/type/Material.h>
#include <LoMap/type/MD5.h>
#include <nox>

namespace nox::app
{
    class MapProvider
    {
    public:
        __asynchronous_thread
        void Update(const std::string & source);

        MD5<type::Map> Produce();

    private:
        Material<std::string> _source;
        MD5<type::Map> _map;
    };
}