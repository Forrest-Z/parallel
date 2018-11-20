/**
 * @file ControlLine.h
 */

#pragma once

#include <nox>

namespace nox::app
{
    struct ControlLine
    {
        std::vector<Ptr<Lane>> segments;
        bool passable = true;
        std::vector<Position> stopPoints;
    };
}