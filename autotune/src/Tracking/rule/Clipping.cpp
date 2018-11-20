#include <Tracking/rule/Clipping.h>

namespace nox::app::rule
{
    double Clipping::Apply(double raw)
    {
        double d = raw - _last_data;

        if(d > 0 and _increase_limit > 0)
            d = std::min(d, _increase_limit);

        if(d < 0 and _decrease_limit > 0)
            d = std::max(d, -_decrease_limit);

        _last_data += d;
        return _last_data;
    }

    Clipping::Clipping(double limit, double init_data)
        : _last_data(init_data), _increase_limit(limit), _decrease_limit(limit)
    {}

    Clipping::Clipping(double increase_limt, double decrease_limit, double init_data)
        : _last_data(init_data), _increase_limit(increase_limt), _decrease_limit(decrease_limit)
    {}
}

