#include <LoMap/provider/ObstacleProvider.h>
#include <LoMap/LoMapConfig.h>
#include <iostream>
USING_NAMESPACE_NOX;
using namespace std;

namespace nox::app
{

    void ObstacleProvider::Update(const nox_msgs::ObstacleArray & obstacles, bool is_global)
    {
        if(is_global)
            _Update(obstacles, _input_global);
        else
            _Update(obstacles, _input_local);
    }

    MD5<vector<Obstacle>> ObstacleProvider::Produce(VehicleStateProvider & state_provider)
    {
        if(_input_local.IsFresh() or _input_global.IsFresh())
        {
            Process(state_provider);
        }

        return _output;
    }

    void ObstacleProvider::Initialize()
    {
        auto & device_params = cache::ReadDeviceParameter();

        if(not device_params.Lidar.empty())
        {
            auto & lidar = device_params.Lidar[0];

            _device._lidar.Set(
                Position(lidar.x, lidar.y, lidar.z),
                Rotation(Degree(lidar.yaw), Degree(lidar.pitch), Degree(lidar.roll))
            );

            _device._lidar = _device._lidar.Inverse();

            Logger::D("DynamicSceneUpdater") << "Lidar pose: " << _device._lidar.ToString();
        }
    }

    void ObstacleProvider::Process(VehicleStateProvider & state_provider)
    {
        _output.reset({}, Clock::us());

        if(_input_local.IsInit())
        {
            auto input = _input_local.Get().data();
            Time time;
            time.From(input.header);

            auto state = state_provider.Produce(time);
            auto current_pose = state.data().pose;

            for(const auto & i : _input_local.Get().data().obstacles)
            {
                _output.data().emplace_back();
                _output.data().back().From(i);
                _output.data().back() *= (current_pose * _device._lidar);
            }
        }

        if(_input_global.IsInit())
        {
            for(const auto & i : _input_global.Get().data().obstacles)
            {
                _output.data().emplace_back();
                _output.data().back().From(i);
            }
        }
    }

    void ObstacleProvider::_Update(const nox_msgs::ObstacleArray &src, Material<MD5<nox_msgs::ObstacleArray>> &dst)
    {
        MD5<nox_msgs::ObstacleArray> input(src);
        input.md5() = Clock::us();
        dst.Update(input);
    }


}
