#include <Tracking/rule/Clipping.h>

namespace nox::app::rule
{
    double Clipping::Apply(double raw)
    {
        double d = raw - _last_data;
        double dsec = _timer.Interval().Get<Second>();
        dsec = std::min(dsec, 2 * _interval);

        double scale = dsec / _interval;
        double increase_limit = _increase_limit * scale;
        double decrease_limit = _decrease_limit * scale;

        if(d > 0 and increase_limit > 0)
            d = std::min(d, increase_limit);

        if(d < 0 and decrease_limit > 0)
            d = std::max(d, -decrease_limit);

        _last_data += d;
        return _last_data;
    }

    Clipping::Clipping(type::Time interval, double limit, double init_data)
        : _interval(interval.Get<Second>()), _last_data(init_data), _increase_limit(limit), _decrease_limit(limit)
    {
        _timer.Start();
    }

    Clipping::Clipping(type::Time interval, double increase_limt, double decrease_limit, double init_data)
        : _interval(interval.Get<Second>()), _last_data(init_data), _increase_limit(increase_limt), _decrease_limit(decrease_limit)
    {
        _timer.Start();
    }

    Clipping::Clipping()
    {
        _timer.Start();
    }
}

