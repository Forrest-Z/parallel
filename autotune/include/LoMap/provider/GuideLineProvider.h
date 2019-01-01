/**
 * @file GuideLineProvider.h，引导线提供商
 */

#pragma once

#include <LoMap/pattern/SupplyChain.h>
#include <LoMap/type/ControlLine.h>
#include <optional>
#include <LoMap/type/MD5.h>
#include <LoMap/type/Material.h>

namespace nox::app
{
    class GuideLineProvider
    {
    public:
        MD5<vector<Ptr<GuideLine>>> Produce(const MD5<type::Map> & map, const MD5<Odometry> & vehicle_state);

    private: /// 生成半成品引导线
        void Update();

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
        vector<Ptr<ControlLine>> GenerateControlLines(
            Ptr<Road> road,
            const vector<int> &path = {},
            vector<int> *end_index = nullptr);

        /**
         * 遍历路口处一个路连接的多个Lane，产生GuideLine
         * @param roadLink 路口的路连接
         * @param begin_index 入口选择，若有选择，则结果仅有一条GuideLine
         * @param next_index 对应每一种可能的GudieLine的结束时的下一条车道索引
         * @return 每一种可能的GuideLine
         */
        vector<Ptr<ControlLine>> GenerateControlLines(
            Ptr<RoadLink> roadLink,
            std::optional<int> begin_index = std::optional<int>(),
            vector<int> * next_index = nullptr);

        void AddGuideLine(Ptr<ControlLine> controlLine);

        void AddGuideLines(const vector<Ptr<ControlLine>> & controlLines);

        void AppendControlLine(Ptr<ControlLine> src, Ptr<ControlLine> extra);

        void SetEndLine(Ptr<ControlLine> controlLine);

        void SetEndLines(const vector<Ptr<ControlLine>> & controlLines);

    private: /// 生成细化的引导线
        void Generate();

        void BuildBoundary();

    private:
        Material<MD5<type::Map>>    _hdmap;
        Material<MD5<Odometry>>     _vehicle_state;

        struct SFP // 半成品
        {
            Ptr<ControlLine> _controlLine;
            Ptr<type::DiscretePath> _path;
        };

        vector<SFP>                 _sfps;
        MD5<vector<Ptr<GuideLine>>> _guideLines;
    };
}