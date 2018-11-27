#include <LoMap/provider/VehicleStateProvider.h>
USING_NAMESPACE_NOX;
namespace nox::app
{

    void VehicleStateProvider::Update(const Odometry &state)
    {
        _input.Update(state);
    }

    MD5<Odometry> VehicleStateProvider::Produce()
    {
        static std::hash<long> hasher;

        if(_input.IsFresh())
        {
            _output.reset(_input.Get(), hasher(Clock::us()));
        }

        return _output;
    }

    MD5<Odometry> VehicleStateProvider::Produce(Time time)
    {
        return Produce(); // TODO: 需要评估时间
    }
}

