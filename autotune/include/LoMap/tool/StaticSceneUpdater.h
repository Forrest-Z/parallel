/**
 * @brief 地图数据解析器，作为一个中间转换工具之用
 * @details
 * 1. 仅处理与地图数据相关的东西；
 * 2. 不会维护无关地图的数据联系（如障碍物）
 */
#pragma once

#include <nox>
#include <old_nox_msgs.h>
#include <std_msgs/String.h>
#include <optional>
#include "SceneObjectCreator.h"
using namespace nox::type;
using std::vector;
using std::optional;

namespace nox::app
{
    class StaticSceneUpdater
    {
    public:
        explicit StaticSceneUpdater(Ptr<Scene> scene);

        void Update(const nox_msgs::Road &source);

        void Update(const std_msgs::String & source);

    private:
        struct ControlLine
        {
            std::vector<Ptr<Lane>> segments;
        };

    private:
        void ClearSceneObjects();

        void Update(Ptr<Road> road);

        void Update(Ptr<RoadLink> roadLink);

        void Update(Ptr<Road> road, Ptr<RoadLink> roadLink);

        void Update(Ptr<RoadLink> roadLink, Ptr<Road> road);

        void Update(Ptr<Road> in_road, Ptr<RoadLink> roadLink, Ptr<Road> out_road);

        /**
         * 遍历路中的每一前后车道连接，产生GuideLine（过程中应用部分交规）
         * @param road 路的指针，包含多个RoadSection，每个RoadSection包含多个Lane
         * @param path 初始车道选择（最多包含一个点）
         * @param end_index 每一种可能结束时的车道索引，用于下一段的选择初始化
         * @return 每一种可能的Guide Line
         */
        vector<Ptr<GuideLine>> GenerateGuideLines(
            Ptr<Road> road,
            const vector<int> & path = {},
            vector<int> * end_index = nullptr);

        /**
         * 遍历路口处一个路连接的多个Lane，产生GuideLine
         * @param roadLink 路口的路连接
         * @param begin_index 入口选择，若有选择，则结果仅有一条GuideLine
         * @param next_index 对应每一种可能的GudieLine的结束时的下一条车道索引
         * @return 每一种可能的GuideLine
         */
        vector<Ptr<GuideLine>> GenerateGuideLines(
            Ptr<RoadLink> roadLink,
            optional<int> begin_index = optional<int>(),
            vector<int> * next_index = nullptr);

        void AddGuideLine(Ptr<GuideLine> guideLine);

        void AddGuideLines(const vector<Ptr<GuideLine>> & guideLines);

        void AppendGuideLine(Ptr<GuideLine> src, Ptr<GuideLine> extra);

    private:
        Ptr<Scene> _scene;
        Map _hdmap;
    };


}