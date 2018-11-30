#include <LoMap/tool/SceneGenerator.h>

namespace nox::app
{

    void SceneGenerator::OnRun()
    {
        /// 1. 获取高精地图
        auto map = _map_provider.Produce();

        /// 2. 从地图计算引导线
        auto guideLines = _guide_line_provider.Produce(map);

        /// 3. 获取当前车的姿态
        auto vehicle_state = _vehicle_state_provider.Produce();

        /// 4. 获得障碍物信息
        auto obstacles = _obstacle_provider.Produce(_vehicle_state_provider);

        /// 5. 附上红绿灯信息
        guideLines = _traffic_light_provider.Produce(map, guideLines);

        /// 6. 处理所有停止线信息
        _stop_line_provider.Produce(guideLines);

        /// 7. 将所有信息整合进ready scene
        Locking(_ready_scene)
        {
            _scene_provider.Produce(guideLines, obstacles, vehicle_state, _ready_scene);
        }
    }

    nox_msgs::Scene SceneGenerator::GetMsg() const
    {
        Locking(_ready_scene)
        {
            nox_msgs::Scene result;
            _ready_scene.To(result);
            return result;
        }
    }

    void SceneGenerator::UpdateMap(const std::string &source)
    {
        _map_provider.Update(source);
    }

    void SceneGenerator::UpdateVehicleState(const type::Odometry &state)
    {
        _vehicle_state_provider.Update(state);
    }

    void SceneGenerator::OnStart()
    {
        _obstacle_provider.Initialize();
        _vehicle_state_provider.Initialize();
    }

    void SceneGenerator::UpdateObstacles(const nox_msgs::ObstacleArray &obstacles, bool is_global)
    {
        _obstacle_provider.Update(obstacles, is_global);
    }

    void SceneGenerator::UpdateTrafficLight(const type::Signal &signal)
    {
        _traffic_light_provider.Update(signal);
    }
}

