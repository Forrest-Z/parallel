#include <LoMap/tool/StaticSceneUpdater.h>
#include <tuple>
USING_NAMESPACE_NOX;
using namespace nox::app;
using namespace std;

StaticSceneUpdater::StaticSceneUpdater(Ptr<Scene> scene)
{
    assert(scene);
    _scene = scene;
}

void StaticSceneUpdater::Update(const nox_msgs::Road &source)
{
    ClearSceneObjects();
}

void StaticSceneUpdater::ClearSceneObjects()
{
    _scene->Clear();
}

void StaticSceneUpdater::Update(const std_msgs::String &source)
{
    _scene->GuideLines.clear();
    _hdmap.From(xml::Node(source.data));

    // 假设高精度地图只会传来最多一个路口，路口不会有路方向的歧义
    if(_hdmap.Junctions.empty())
    {
        assert(_hdmap.Roads.size() == 1);
        auto road = _hdmap.Roads.begin()->second;
        Update(road);
    }
    else
    {
        assert(_hdmap.Junctions.size() == 1);
        auto junction = _hdmap.Junctions.begin()->second;
        assert(junction->RoadLinks.size() == 1);
        auto connection = junction->RoadLinks.begin()->first;
        auto roadLink = junction->RoadLinks.begin()->second;

        bool has_from_lane = _hdmap.Roads.find(connection.from) == _hdmap.Roads.end();
        bool has_to_lane = _hdmap.Roads.find(connection.to) == _hdmap.Roads.end();

        using tb = std::tuple<bool, bool>;
        Switch(tb(has_from_lane, has_to_lane))
            Case(tb(true, true))
            {
                Update(_hdmap.Roads[connection.from], roadLink, _hdmap.Roads[connection.to]);
            }
            Case(tb(true, false))
            {
                Update(_hdmap.Roads[connection.from], roadLink);
            }
            Case(tb(false, true))
            {
                Update(roadLink, _hdmap.Roads[connection.to]);
            }
            Case(tb(false, false))
            {
                Update(roadLink);
            }
        EndSwitch()
    }
}

void StaticSceneUpdater::Update(Ptr<Road> road)
{
    auto guideLines = GenerateGuideLines(road);
    AddGuideLines(guideLines);
}

void StaticSceneUpdater::Update(Ptr<RoadLink> roadLink)
{
    auto guideLines = GenerateGuideLines(roadLink);
    AddGuideLines(guideLines);
}

void StaticSceneUpdater::Update(Ptr<Road> road, Ptr<RoadLink> roadLink)
{
    //region 选择通过第一个路段的所有车道
    vector<int> end_index;
    auto guideLines = GenerateGuideLines(road, {}, &end_index);
    //endregion

    for(size_t i = 0, end = end_index.size(); i < end; ++i)
    {
        //region 选择通过路口的所有车道
        auto junction_guideLines = GenerateGuideLines(roadLink, end_index[i]);
        assert(junction_guideLines.size() <= 1);
        //endregion

        if(junction_guideLines.empty())
            guideLines[i]->passable = false; // 追加cost
        else
            AppendGuideLine(guideLines[i], junction_guideLines[0]);
    }

    AddGuideLines(guideLines);
}

void StaticSceneUpdater::Update(Ptr<RoadLink> roadLink, Ptr<Road> road)
{
    //region 选择通过路口的车道
    vector<int> next_index;
    auto guideLines = GenerateGuideLines(roadLink, optional<int>(), &next_index);
    //endregion

    for(size_t i = 0, end = guideLines.size(); i < end; ++i)
    {
        //region 选择所有通过下一个路段的所有车道
        vector<int> path{next_index[i]};
        auto next_guideLines = GenerateGuideLines(road, path);
        assert(not next_guideLines.empty());
        //endregion

        //region 将上述车道与通过路口的车道各个连接成一条车道
        for(auto & j : next_guideLines)
        {
            auto longer_guideLine = New<GuideLine>(*guideLines[i]);
            AppendGuideLine(longer_guideLine, j);
            AddGuideLine(longer_guideLine);
        }
        //endregion
    }
}

void StaticSceneUpdater::Update(Ptr<Road> in_road, Ptr<RoadLink> roadLink, Ptr<Road> out_road)
{
    //region 产生通过第一个路段的所有车道
    vector<int> end_index;
    auto guideLines = GenerateGuideLines(in_road, {}, &end_index);
    //endregion

    for(size_t i = 0, end = guideLines.size(); i < end; ++i)
    {
        //region 选择通过路口的各个车道（可能不存在）
        vector<int> next_index;
        auto junction_guideLines = GenerateGuideLines(roadLink, end_index[i], &next_index);
        assert(junction_guideLines.size() <= 1);
        //endregion

        if(junction_guideLines.empty())
        {
            guideLines[i]->passable = false; // 追加cost
            AddGuideLine(guideLines[i]);
        }
        else
        {
            //region 追加通过路口的车道，并且选择从该车道通过下一个路段的所有车道
            AppendGuideLine(guideLines[i], junction_guideLines[0]);

            vector<int> path{next_index[i]};
            auto next_guideLines = GenerateGuideLines(out_road, path);
            assert(not next_guideLines.empty());
            //endregion

            //region 将通过下一个路段的所有车道与通过上一个路口和路段的车道全部连接起来
            for(auto & j : next_guideLines)
            {
                auto longer_guideLine = New<GuideLine>(*guideLines[i]);
                AppendGuideLine(longer_guideLine, j);
                AddGuideLine(longer_guideLine);
            }
            //endregion
        }
    }
}

void StaticSceneUpdater::AddGuideLine(Ptr<GuideLine> guideLine)
{
    static scene::ID id = 0;
    _scene->GuideLines[id++] = guideLine;
}

vector<Ptr<GuideLine>> StaticSceneUpdater::GenerateGuideLines(Ptr<Road> road, const vector<int> & path_, vector<int> * end_index)
{
    vector<Ptr<GuideLine>> result;

    /// 重构车道序列，构造GuideLine
    auto Reconstruct = [&](const std::vector<int> & path)
    {
        auto guideLine = New<GuideLine>();
        Range s(0, 0);

        //region 根据path序列追加各个车道
        bool is_illegal = false;

        for(size_t i = 0, end = path.size(); i < end; ++i)
        {
            auto section = road->Sections[i];
            if(section->Lanes.find(path[i]) == section->Lanes.end())
            {
                is_illegal = true;
                break;
            }

            auto lane = section->Lanes[path[i]];
            s.End = s.Start + lane->Length();

            guideLine->path += lane->Discretize(0.5);
            guideLine->speedLimits.emplace_back(s, lane->speedLimit);
            s.Start = s.End;
        }

        if(is_illegal) return; // 如果存在非法路径（经过不存在车道），则不添加该结果
        //endregion

        if(end_index)
            end_index->push_back(path.back());
        result.push_back(guideLine);
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
        for(auto & i : section->Lanes) // 遍历当前section可走的车道
        {
            auto & curr_lane = i.second;
            int curr_lane_index = i.first;
            int last_lane_index = path.back();

            if(index == 0)
            {
                //region 第一块路段，全追加进搜索目录
                path.push_back(curr_lane_index);
                Search(path);
                path.pop_back();
                //endregion
            }
            else
            {
                auto last_lane = road->Sections[index - 1]->Lanes[last_lane_index];
                bool is_finished = true;

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

                //region 如果找不到下一个连接车道，则重构已有车道
                if(is_finished)
                {
                    Reconstruct(path);
                }
                //endregion
            }
        }
        //endregion
    };

    vector<int> temp = path_;
    Search(temp);
    return result;
}

vector<Ptr<GuideLine>> StaticSceneUpdater::GenerateGuideLines(Ptr<RoadLink> roadLink, optional<int> begin_index, vector<int> *next_index)
{
    vector<Ptr<GuideLine>> result;

    for(auto & i : roadLink->LaneLinks)
    {
        auto connection = i.first;
        auto lane = i.second;

        //region 如果没有指定begin_index，则遍历全部，否则只处理指定的
        if(!begin_index or begin_index.value() == connection.from)
        {
            auto guideLine = New<GuideLine>();

            guideLine->path = lane->Discretize(0.5);
            guideLine->speedLimits.emplace_back(Range(0, lane->Length()), lane->speedLimit);

            if(next_index)
                next_index->push_back(connection.to);
            result.push_back(guideLine);

            if(begin_index) break; // 若有指定begin_index，则仅push此条选择
        }
        //endregion
    }

    return result;
}

void StaticSceneUpdater::AppendGuideLine(Ptr<GuideLine> src, Ptr<GuideLine> extra)
{
    //region 追加路径
    src->path += extra->path;
    //endregion

    //region 追加速度控制
    double s0 = 0;
    if(not src->speedLimits.empty())
        s0 = src->speedLimits.back().s.End;

    size_t i = src->speedLimits.size();
    src->speedLimits.insert(src->speedLimits.begin(), extra->speedLimits.begin(), extra->speedLimits.end());
    for(size_t end = src->speedLimits.size(); i < end; ++i)
    {
        src->speedLimits[i].s.Start += s0;
        src->speedLimits[i].s.End += s0;
    }
    //endregion

    //region 更新停止线
    if(not src->stopLine)
        src->stopLine = extra->stopLine;
    //endregion
}

void StaticSceneUpdater::AddGuideLines(const vector<Ptr<GuideLine>> &guideLines)
{
    for(auto i : guideLines)
        AddGuideLine(i);
}


