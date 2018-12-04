#include <Tracking/component/SimpleLongitudinalController.h>
#include <iostream>
USING_NAMESPACE_NOX;
using namespace nox::app;

double SimpleLongitudinalController::Calculate(const type::Trajectory &path, const type::Vehicle &vehicle)
{
    auto matched_point = path.PointAtPosition(vehicle.pose.t);
    Logger::I("Lon").Print("(v, w): %lf, %lf", matched_point.v, matched_point.a);
    return matched_point.v;
}
