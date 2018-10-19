#include <LoMap/tool/StaticSceneUpdater.h>
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

    vector<Ptr<Lane>> lanes;
    Ptr<Junction> junction = _junction_creator.Create();
    bool last_in_junction = false;

    for(auto & block : source.blocks)
    {
        size_t last_segment_size = lanes.size();
        size_t next_segment_size = block.points.size();
        assert(next_segment_size > 0);

        /// 每次block里边的点前后数量不一致时，说明到了路口，或者车道汇合处
        if(last_segment_size != next_segment_size)
        {
            //region 将上一个路段的路加入数据中
            for(auto & i : lanes)
            {
                _scene->lanes[i->id] = i;
            }
            //endregion

            //region 创建新的路段
            auto lane_segment = _lane_segment_creator.Create();
            scene::ID target_id = scene::UNDEFINED_ID;
            lanes.clear();

            for(size_t i = 0; i < next_segment_size; ++i)
            {
                auto lane = _lane_creator.Create();
                lane->minSpeed = block.minSpeed;
                lane->maxSpeed = block.maxSpeed;
                lane->width = param.default_lane_width;
                lane->segmentID = lane_segment->id;

                lane_segment->insert(lane->id);
                lanes.push_back(lane);

                if(i == block.target_index)
                    target_id = lane->id;
            }

            _scene->laneSegments[lane_segment->id] = lane_segment;

            bool next_in_junction = block.target_index == -1;
            if(next_in_junction) // 此为路口的状态，假设路口有且仅有一个车道
                target_id = lanes[0]->id;
            //endregion

            //region 更新路口状态
            if(last_segment_size == 0) /// 第一个点
            {
                last_in_junction = next_in_junction;
                if(last_in_junction)
                {
                    junction->from = scene::Unknown; // 第一个点，没有上一个路段的信息
                    junction->through = target_id;
                }
                else
                {
                    junction->from = target_id;
                }
            }
            else if(last_in_junction) /// 如果当前处于路口，切换路段意味着结束了一个路口生成
            {
                junction->to = target_id;
                _scene->junctions[junction->id] = junction;
                junction = _junction_creator.Create();
                junction->from = target_id;
            }
            else if(next_in_junction) /// 如果当前没有在路口，但是下一个点在路口，则更新路口状态，成为【在路口】
            {
                last_in_junction = true;
                junction->through = target_id; // 假设在路口时，有且仅有一条车道
            }
            else /// 其他情况，此为非路口却发生路段切换，是为车道合并或分离
            {
                junction->from = target_id;
            }
            //endregion
        }

        /// 将当前block的路点加入到相应的Lane中
        assert(lanes.size() == block.points.size());
        for(auto & [p, lane] : together(block.points, lanes))
        {
            PathPoint pathPoint;
            pathPoint.pose.Set(Position(p.x, p.y, p.z), Rotation(p.yaw));
            lane->path.Add(pathPoint);
        }
    }

    /// 将最后的一段路加入scene中
    for(auto & i : lanes)
    {
        _scene->lanes[i->id] = i;
    }

    /// 对路进行平滑计算，并计算出kappa，dkappa等信息
    tool::Smoother smoother;

    for(auto & it : _scene->lanes)
    {
        it.second->path = smoother.Smooth(it.second->path);
    }
}

void StaticSceneUpdater::ClearSceneObjects()
{
    _scene->lanes.clear();
    _scene->laneSegments.clear();
    _scene->junctions.clear();
    _scene->signalLights.clear();
}



