#include <Tracking/tool/SimpleLongitudinalController.h>
USING_NAMESPACE_NOX;
using namespace nox::app;

double SimpleLongitudinalController::Calculate(const type::Trajectory &path, const type::Vehicle &vehicle)
{
    auto nearest_index = path.QueryNearestByPosition(vehicle.pose.t);
    auto nearest_point = path[nearest_index];

    return nearest_point.v;
    // return math::Clamp(nearest_point.v, 5.0 / 3.6, 15 / 3.6);
}
