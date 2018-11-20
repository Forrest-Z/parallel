#include <Tracking/rule/Custom.h>

namespace nox::app::rule
{

    Custom::Custom(const Custom::TFunction &func)
        : _func(func)
    {}

    double Custom::Apply(double raw)
    {
        return _func ? _func(raw) : raw;
    }
}

