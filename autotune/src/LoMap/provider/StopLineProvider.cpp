#include <LoMap/provider/StopLineProvider.h>

namespace nox::app
{
    const std::string StopLineProvider::TrafficLight("Traffic Light");
    const std::string StopLineProvider::DeadEnd("Dead End");

    void StopLineProvider::Produce(MD5<vector<Ptr<nox::type::GuideLine>>> &guideLines)
    {
        for(auto & i : guideLines.data())
        {
            //region 初始化
            i->ClearStopLine();
            auto end_it = i->stopLines.end();
            //endregion

            //region 处理红绿灯的停止线
            auto traffic_light_it = i->stopLines.find(TrafficLight);
            if(traffic_light_it != end_it)
            {
                for(auto & j : *traffic_light_it->second)
                    i->SetStopLine(j);
            }
            //endregion

            //region 处理死路的停止线
            auto dead_end_it = i->stopLines.find(DeadEnd);
            if(dead_end_it != end_it)
            {
                for(auto & j : *dead_end_it->second)
                    i->SetStopLine(j);
            }
            //endregion
        }
    }
}
