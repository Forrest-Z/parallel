/**
 * @file ControlLine.h
 */

#pragma once

#include <nox>

namespace nox::app
{
    struct ControlLine
    {
        std::vector<nox::Ptr<type::RoadSection>> sections;
        std::vector<nox::Ptr<type::Lane>> segments;
        bool                       passable = true;
        std::vector<type::Position>     stopPoints;
        size_t           junction_begin_index = -1;    // 标记segments第n个Lane为到路口前的最后一个lane
        size_t             junction_end_index = -1;    // 标记segments地m个Lane为出路口前的最后一个lane
    };
}