#include <Tracking/rule/Limiting.h>

namespace nox::app::rule
{

    Limiting::Limiting(double min, double max)
    {
        if(min > max)
            std::swap(min, max);

        _min = min;
        _max = max;
    }

    double Limiting::Apply(double raw)
    {
        return math::Clamp(raw, _min, _max);
    }
}

