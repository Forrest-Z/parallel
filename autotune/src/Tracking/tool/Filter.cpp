#include <Tracking/tool/Filter.h>

namespace nox::app
{
    double Filter::operator()(double raw)
    {
        double t = raw;
        for(auto & i : _rules)
        {
            t = i->Apply(t);
        }
        return t;
    }
}

