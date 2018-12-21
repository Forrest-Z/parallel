//
// Created by yarten on 18-12-19.
//

#pragma once

#include <optional>
#include <LoMap/type/MD5.h>
#include <LoMap/type/Material.h>
#include <old_nox_msgs.h>
#include <nox>

namespace nox::app
{
    class OldMapProvider
    {
    public:
        __asynchronous_thread
        void Update(const nox_msgs::Road & road);

        MD5<vector<Ptr<GuideLine>>> Produce();

    private:
        struct SpeedSegment
        {
            Position begin, end;
            Bound speed;
        };

        void AddSpeedControl(const vector<SpeedSegment> & speeds, GuideLine & guideLine) const;

        void AddDeadEnd(GuideLine & guideLine) const;

        void AddBoundary(GuideLine & guideLine) const;

    private:
        Material<MD5<nox_msgs::Road>> _old_map;
        MD5<vector<Ptr<GuideLine>>>   _guideLines;
    };
}