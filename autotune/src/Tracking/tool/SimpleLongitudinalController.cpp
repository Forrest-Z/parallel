#include <Tracking/tool/SimpleLongitudinalController.h>
#include <iostream>
USING_NAMESPACE_NOX;
using namespace nox::app;

double SimpleLongitudinalController::Calculate(const type::Trajectory &path, const type::Vehicle &vehicle)
{
    auto matched_point = path.PointAtPosition(vehicle.pose.t);
    return matched_point.v;
}
