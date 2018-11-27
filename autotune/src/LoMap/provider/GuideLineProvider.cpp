#include <LoMap/provider/GuideLineProvider.h>
#include <LoMap/tool/GuideLineBuilder.h>
USING_NAMESPACE_NOX;
using namespace std;

namespace nox::app
{
    MD5<vector<Ptr<nox::type::GuideLine>>> GuideLineProvider::Produce(const MD5<type::Map> &map)
    {
        _hdmap.Update(map);

        if(_hdmap.IsFresh())
        {
            Update();
            Logger::D("GuideLineProvider") << "Generate Guide Lines: " << _guideLines.data().size();
        }

        return _guideLines;
    }

    void GuideLineProvider::Update()
    {
        auto map = _hdmap.Get();
        auto hdmap = map.data();
        _guideLines.reset({}, map.md5());

        for(auto & i : hdmap.Roads)
        {
            auto & road = i.second;
            auto & sections = road->Sections;
            if(road->direction == -1)
            {
                std::reverse(sections.begin(), sections.end());
                for(auto & j : sections)
                {
                    for(auto & k : j->Lanes)
                    {
                        auto & lane = k.second;
                        std::swap(lane->predecessors, lane->successors);
                    }
                }
            }
        }

        // 假设高精度地图只会传来最多一个路口，路口不会有路方向的歧义
        if(hdmap.Junctions.empty())
        {
            if(not hdmap.Roads.empty())
            {
                assert(hdmap.Roads.size() == 1);
                auto road = hdmap.Roads.begin()->second;
                Update(road);
            }
        }
        else
        {
            assert(hdmap.Junctions.size() == 1);
            auto junction = hdmap.Junctions.begin()->second;
            assert(junction->RoadLinks.size() == 1);
            auto connection = junction->RoadLinks.begin()->first;
            auto roadLink = junction->RoadLinks.begin()->second;

            bool has_from_lane = hdmap.Roads.find(connection.from) != hdmap.Roads.end();
            bool has_to_lane = hdmap.Roads.find(connection.to) != hdmap.Roads.end();

            using tb = std::tuple<bool, bool>;
            Switch(tb(has_from_lane, has_to_lane))
                Case(tb(true, true))
                {
                    Update(hdmap.Roads[connection.from], roadLink, hdmap.Roads[connection.to]);
                }
                Case(tb(true, false))
                {
                    Update(hdmap.Roads[connection.from], roadLink);
                }
                Case(tb(false, true))
                {
                    Update(roadLink, hdmap.Roads[connection.to]);
                }
                Case(tb(false, false))
                {
                    Update(roadLink);
                }
            EndSwitch()
        }
    }

    void GuideLineProvider::Update(Ptr<Road> road)
    {
        auto controlLines = GenerateControlLines(road);
        for(auto & i : controlLines)
            SetEndLine(i);
        AddGuideLines(controlLines);
    }

    void GuideLineProvider::Update(Ptr<RoadLink> roadLink)
    {
        auto controlLines = GenerateControlLines(roadLink);
        AddGuideLines(controlLines);
    }

    void GuideLineProvider::Update(Ptr<Road> road, Ptr<RoadLink> roadLink)
    {
        //region 选择通过第一个路段的所有车道
        vector<int> end_index;
        auto controlLines = GenerateControlLines(road, {}, &end_index);
        //endregion

        for(size_t i = 0, end = end_index.size(); i < end; ++i)
        {
            //region 选择通过路口的所有车道
            auto junction_controlLines = GenerateControlLines(roadLink, end_index[i]);
            assert(junction_controlLines.size() <= 1);
            //endregion

            if(junction_controlLines.empty())
                controlLines[i]->passable = false; // 追加cost
            else
                AppendControlLine(controlLines[i], junction_controlLines[0]);
        }

        AddGuideLines(controlLines);
    }

    void GuideLineProvider::Update(Ptr<RoadLink> roadLink, Ptr<Road> road)
    {
        //region 选择通过路口的车道
        vector<int> next_index;
        auto controlLines = GenerateControlLines(roadLink, optional<int>(), &next_index);
        //endregion

        for(size_t i = 0, end = controlLines.size(); i < end; ++i)
        {
            //region 选择所有通过下一个路段的所有车道
            vector<int> path{next_index[i]};
            auto next_controlLines = GenerateControlLines(road, path);
            assert(not next_controlLines.empty());
            //endregion

            //region 将上述车道与通过路口的车道各个连接成一条车道
            for(auto & j : next_controlLines)
            {
                auto longer_controlLine = New<ControlLine>(*controlLines[i]);
                AppendControlLine(longer_controlLine, j);
                AddGuideLine(longer_controlLine);
            }
            //endregion
        }
    }

    void GuideLineProvider::Update(Ptr<Road> in_road, Ptr<RoadLink> roadLink, Ptr<Road> out_road)
    {
        //region 产生通过第一个路段的所有车道
        vector<int> end_index;
        auto controlLines = GenerateControlLines(in_road, {}, &end_index);
        //endregion

        for(size_t i = 0, end = controlLines.size(); i < end; ++i)
        {
            //region 选择通过路口的各个车道（可能不存在）
            vector<int> next_index;
            auto junction_controlLines = GenerateControlLines(roadLink, end_index[i], &next_index);
            assert(junction_controlLines.size() <= 1);
            //endregion

            if(junction_controlLines.empty())
            {
                controlLines[i]->passable = false; // 追加cost
                AddGuideLine(controlLines[i]);
            }
            else
            {
                //region 追加通过路口的车道，并且选择从该车道通过下一个路段的所有车道
                AppendControlLine(controlLines[i], junction_controlLines[0]);

                vector<int> path{next_index[0]};
                auto next_controlLines = GenerateControlLines(out_road, path);
                assert(not next_controlLines.empty());
                //endregion

                //region 将通过下一个路段的所有车道与通过上一个路口和路段的车道全部连接起来
                for(auto & j : next_controlLines)
                {
                    auto longer_controlLine = New<ControlLine>(*controlLines[i]);
                    AppendControlLine(longer_controlLine, j);
                    AddGuideLine(longer_controlLine);
                }
                //endregion
            }
        }
    }

    void GuideLineProvider::AddGuideLine(Ptr<ControlLine> controlLine)
    {
        static scene::ID id = 0;

        auto guideLine = New<GuideLine>();
        guideLine->id = id;
        guideLine->passable = controlLine->passable;

        GuideLineBuilder::BuildPathUsingSpline2(controlLine, guideLine);
        GuideLineBuilder::BuildStopLine(controlLine, guideLine);

        _guideLines.data().push_back(guideLine);
    }

    vector<Ptr<ControlLine>> GuideLineProvider::GenerateControlLines(Ptr<Road> road, const vector<int> &path_,
                                                                      vector<int> *end_index)
    {
        vector<Ptr<ControlLine>> result;

        /// 重构车道序列，构造GuideLine
        auto Reconstruct = [&](const std::vector<int> & path)
        {
            //region 根据path序列追加各个车道
            auto controlLine = New<ControlLine>();
            bool is_illegal = path.empty();

            for(size_t i = 0, end = path.size(); i < end; ++i)
            {
                auto section = road->Sections[i];
                if(section->Lanes.find(path[i]) == section->Lanes.end())
                {
                    is_illegal = true;
                    break;
                }

                auto lane = section->Lanes[path[i]];

                controlLine->segments.push_back(lane);
            }

            if(is_illegal) return; // 如果存在非法路径（经过不存在车道），则不添加该结果
            //endregion

            if(end_index)
                end_index->push_back(path.back());

            result.push_back(controlLine);
        };

        /// 搜索路径
        function<void(std::vector<int> &)> Search = [&](std::vector<int> & path)
        {
            size_t index = path.size();

            //region 当搜索到最后一个路段时，需重构路径
            if(index >= road->Sections.size())
            {
                Reconstruct(path);
                return;
            }
            //endregion

            //region 追加下一个路段
            auto section = road->Sections[index];
            bool is_finished = true; // 当前有的路段无法继续走为true

            for(auto & i : section->Lanes) // 遍历当前section可走的车道
            {
                auto & curr_lane = i.second;
                int curr_lane_index = i.first;

                if(index == 0)
                {
                    //region 第一块路段，全追加进搜索目录
                    path.push_back(curr_lane_index);
                    Search(path);
                    path.pop_back();
                    is_finished = false;
                    //endregion
                }
                else
                {
                    int last_lane_index = path.back();
                    auto last_lane = road->Sections[index - 1]->Lanes[last_lane_index];

                    //region 遍历上下条道路的连接关系
                    for(auto & j : curr_lane->predecessors)
                    {
                        if(j == last_lane_index)
                        {
                            //region 仅遍历上一条车道连接的下一条车道
                            path.push_back(curr_lane_index);
                            Search(path);
                            path.pop_back();
                            is_finished = false;
                            break;
                            //endregion
                        }
                    }
                    //endregion
                }
            }

            //region 如果找不到下一个连接车道，则重构已有车道
            if(is_finished)
            {
                Reconstruct(path);
            }
            //endregion
            //endregion
        };

        vector<int> temp = path_;
        Search(temp);
        return result;
    }

    vector<Ptr<ControlLine>> GuideLineProvider::GenerateControlLines(Ptr<RoadLink> roadLink, optional<int> begin_index,
                                                                      vector<int> *next_index)
    {
        vector<Ptr<ControlLine>> result;

        for(auto & i : roadLink->LaneLinks)
        {
            auto connection = i.first;
            auto lane = i.second;

            //region 如果没有指定begin_index，则遍历全部，否则只处理指定的
            if(!begin_index or begin_index.value() == connection.from)
            {
                auto controlLine = New<ControlLine>();
                controlLine->segments.push_back(lane);

                if(next_index)
                    next_index->push_back(connection.to);
                result.push_back(controlLine);

                if(begin_index) break; // 若有指定begin_index，则仅push此条选择
            }
            //endregion
        }

        return result;
    }

    void GuideLineProvider::AppendControlLine(Ptr<ControlLine> src, Ptr<ControlLine> extra)
    {
        src->segments.insert(src->segments.end(), extra->segments.begin(), extra->segments.end());
        src->stopPoints.insert(src->stopPoints.end(), extra->stopPoints.begin(), extra->stopPoints.end());
    }

    void GuideLineProvider::AddGuideLines(const vector<Ptr<ControlLine>> &controlLines)
    {
        for(const auto &i : controlLines)
            AddGuideLine(i);
    }

    void GuideLineProvider::SetEndLine(Ptr<ControlLine> controlLine)
    {
        controlLine->stopPoints.push_back(controlLine->segments.back()->Back().pose.t);
    }


}
