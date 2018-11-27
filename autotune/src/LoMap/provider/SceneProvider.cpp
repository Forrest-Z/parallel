#include <LoMap/provider/SceneProvider.h>
USING_NAMESPACE_NOX;
namespace nox::app
{

    void SceneProvider::Produce(
        const MD5<vector<Ptr<nox::type::GuideLine>>> & guideLines,
        const MD5<vector<Obstacle>>                  & obstacles,
        const MD5<type::Odometry>                    & state,
        type::Scene                                  & scene)
    {
        _guide_lines.Update(guideLines);
        _obstacles.Update(obstacles);
        _vehicle_state.Update(state);

        if(_guide_lines.IsFresh())
        {
            scene.GuideLines.clear();
            size_t id = 0;
            for(auto & i : _guide_lines.Get().data())
            {
                auto guideLine = New<GuideLine>(*i);
                guideLine->id = id;
                scene.GuideLines[id] = guideLine;
                ++id;
            }

            Logger::I("LoMap") << "Update " << scene.GuideLines.size() << " GuideLines.";
        }

        if(_obstacles.IsFresh())
        {
            scene.Obstacles.clear();
            size_t id = 0;
            for(auto & i : _obstacles.Get().data())
            {
                auto obstacle = New<Obstacle>(i);
                obstacle->id = id;
                scene.Obstacles[id] = obstacle;
                ++id;
            }

            Logger::I("LoMap") << "Update " << scene.Obstacles.size() << " Obstacles.";
        }

        if(_vehicle_state.IsFresh())
        {
            if(not scene.EgoVehicle)
                scene.EgoVehicle = New<Vehicle>();
            scene.EgoVehicle->From(_vehicle_state.Get().data());

            Logger::I("LoMap") << "Update Vehicle State.";
        }
    }
}
