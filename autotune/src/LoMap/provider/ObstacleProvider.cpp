#include <LoMap/provider/ObstacleProvider.h>
#include <LoMap/LoMapConfig.h>
USING_NAMESPACE_NOX;
namespace nox::app
{

    void ObstacleProvider::Update(const std::vector<type::Obstacle> & obstacles, bool is_global)
    {
        if(is_global)
            _Update(obstacles, _input_global);
        else
            _Update(obstacles, _input_local);
    }

    MD5<vector<Obstacle>> ObstacleProvider::Produce(const MD5<type::Odometry> &state)
    {
        _state.Update(state);

        if(_state.IsFresh() or _input_local.IsFresh() or _input_global.IsFresh())
        {
            Process();
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

    void ObstacleProvider::Process()
    {
        static std::hash<long> hasher;

        auto state = _state.Get();
        auto current_pose = state.data().pose;

        _output.reset({}, hasher(Clock::us()));

        if(_input_local.IsInit())
        {
            for(auto i : _input_local.Get().data())
            {
                i *= (current_pose * _device._lidar);
                _output.data().push_back(i);
            }
        }

        if(_input_global.IsInit())
        {
            for(const auto & i : _input_global.Get().data())
            {
                _output.data().push_back(i);
            }
        }
    }

    void ObstacleProvider::_Update(const std::vector<type::Obstacle> &src, Material<MD5<vector<Obstacle>>> &dst)
    {
        static std::hash<long> hasher;

        MD5<std::vector<type::Obstacle>> input(src);
        input.md5() = hasher(Clock::us());

        dst.Update(input);
    }


}
