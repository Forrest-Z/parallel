#include <LoMap/tool/SceneOverlapBuilder.h>
#include <map>
USING_NAMESPACE_NOX;
using namespace nox::app;


SceneOverlapBuilder::SceneOverlapBuilder(Ptr<type::Scene> scene)
    : _scene(scene)
{
    assert(scene);
}

void SceneOverlapBuilder::Rebuild()
{
    for(auto & it : _scene->obstacles)
    {
        auto & obstacle = it.second;
        std::map<scene::ID, Overlap> overlaps;

        /// 遍历障碍物每一个时刻所在的位置的重叠情况
        for(auto & point : obstacle->prediction)
        {
            auto box = obstacle->CreateBoundingBox(point);

            /// 遍历每一根Lane，查看障碍物对其覆盖的情况
            for(auto & it2 : _scene->lanes)
            {
                //region 计算重叠区域
                auto & lane = it2.second;
                auto shift = lane->path.FrenetAtPosition(obstacle->pose.t);

                if(abs(shift.l) > lane->width * 0.5)
                    continue;

                vector<Frenet> corners;
                for(auto & corner : box.Corners())
                {
                    corners.push_back(lane->path.FrenetAtPosition(corner));
                }
                //endregion

                //region 创建重叠
                auto overlap = _overlap_creator.Create();
                overlap->type = scene::Lane;
                overlap->on = lane->id;
                overlap->range.Start = corners[0].s;
                overlap->range.End = corners[0].s;

                for(auto & i : corners)
                {
                    overlap->range.Start = std::min(overlap->range.Start, i.s);
                    overlap->range.End = std::max(overlap->range.End, i.s);
                }
                //endregion

                //region 更新障碍物的重叠
                if(overlaps.find(overlap->id) == overlaps.end())
                {
                    overlaps[overlap->id] = *overlap;
                }
                else
                {
                    overlaps[overlap->id].range.Start = std::min(overlaps[overlap->id].range.Start, overlap->range.Start);
                    overlaps[overlap->id].range.End = std::max(overlaps[overlap->id].range.End, overlap->range.End);
                }
                //endregion
            }
        }

        //region 将重叠结果整理进障碍物中
        obstacle->overlaps.clear();
        for(auto & it2 : overlaps)
        {
            obstacle->overlaps.push_back(it2.second);
        }
        //endregion
    }
}
