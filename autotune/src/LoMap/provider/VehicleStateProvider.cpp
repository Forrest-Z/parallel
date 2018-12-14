#include <LoMap/provider/VehicleStateProvider.h>
USING_NAMESPACE_NOX;
namespace nox::app
{

    void VehicleStateProvider::Update(const Odometry &state)
    {
        _input.Update(state);

        Synchronized(this)
            _states.Push(state);
    }

    MD5<Odometry> VehicleStateProvider::Produce()
    {
        if(_input.IsFresh())
        {
            _output.reset(_input.Get(), Clock::us());
        }

        return _output;
    }

    MD5<Odometry> VehicleStateProvider::Produce(Time time)
    {
        static std::hash<double> hasher;

        double time_ = time.Get<Second>();

        Synchronized(this)
        {
            if(time_ == 0 or _states.empty())
                return Produce();
            else
            {
                size_t index = math::BinaryMatch(
                    _states.begin(),
                    _states.end(),
                    time_,
                    [](const Odometry & p, double t)
                    {
                        return p.timeStamp.Get<Second>() < t;
                    },
                    [](const Odometry & lower, const Odometry & upper, double t)
                    {
                        return t - lower.timeStamp.Get<Second>() < upper.timeStamp.Get<Second>() - t;
                    }
                );

                MD5<Odometry> result;
                result.data() = _states[index];
                result.md5() = hasher(time_);
                return result;
            }
        }
    }

    void VehicleStateProvider::Initialize()
    {
        _states.SetBufferSize(100);
    }
}

