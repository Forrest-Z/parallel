#include <LoMap/provider/TrafficLightProvider.h>
USING_NAMESPACE_NOX;

namespace nox::app
{

    void TrafficLightProvider::Update(const type::Signal &traffic_light)
    {
        _signal.Update(traffic_light);
    }

    MD5<vector<Ptr<GuideLine>>>
    TrafficLightProvider::Produce(const MD5<Map> &map, const MD5<vector<Ptr<GuideLine>>> &guideLines)
    {
        _map.Update(map);
        _guide_lines.Update(guideLines);

        if(_signal.IsFresh() or _map.IsFresh() or _guide_lines.IsFresh())
        {
            Process();
        }

        return _output;
    }

    void TrafficLightProvider::Process()
    {
        static std::hash<long> hasher;

        //region 初始化
        auto & map = _map.Get().data();
        auto & guideLines = _guide_lines.Get().data();

        _output.reset(guideLines, hasher(Clock::us()));
        
        if(not _signal.IsInit())
            return;
        auto signal = _signal.Get();
        //endregion

        //region 基本筛查
        if(map.Junctions.empty()) // 没有路口时，忽略红绿灯信息
            return;

        assert(map.Junctions.size() == 1);
        auto junction = map.Junctions.begin()->second;
        assert(junction->RoadLinks.size() == 1);
        auto connection = junction->RoadLinks.begin()->first;
        auto roadLink = junction->RoadLinks.begin()->second;

        if(roadLink->direction bitand signal.direction) // 路口方向与绿灯方向一致时，不需要操作
            return;

        auto road_it = map.Roads.find(connection.from);
        if(road_it == map.Roads.end()) // 若路口没有进入的road时，也不处理
            return;
        //endregion

        //region 取出停止点
        auto road = road_it->second;
        assert(not road->Sections.empty());

        auto section = road->Sections.back();
        assert(not section->Lanes.empty());

        auto lane = section->Lanes.begin()->second;
        assert(lane);

        auto last_point = lane->Back();
        //endregion

        //region 给所有的引导线添加停止线
        for(auto & i : _output.data())
        {
            auto point = i->path.PointAtPosition(last_point.pose.t);
            i->AddStopLine(point.s);
        }
        //endregion
    }
}

