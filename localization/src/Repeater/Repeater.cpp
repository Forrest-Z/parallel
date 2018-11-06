#include <Repeater/Repeater.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

void Repeater::Process(nox_msgs::Location localization, nox_msgs::Chassis chassis,
                       optional<nav_msgs::Odometry> &vehicle_state)
{
    vehicle_state.emplace();

    type::Odometry odometry;
    odometry.pose.x = localization.x;
    odometry.pose.y = localization.y;
    odometry.pose.theta = Degree(localization.yaw + 90).Get<Radian>();

    odometry.v.x = chassis.speed;

    odometry.To(vehicle_state.value());
}
