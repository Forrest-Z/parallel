#include <Repeater/Repeater.h>
#include <Common/Common.h>
using namespace nox::app;
USING_NAMESPACE_NOX;


void Repeater::Process(nox_msgs::Location localization, nox_msgs::Chassis chassis,
                       geometry_msgs::TwistWithCovarianceStamped Velocity, optional<nav_msgs::Odometry> &vehicle_state)
{
    vehicle_state.emplace();

    type::Odometry odometry;
    odometry.pose.x = localization.x;
    odometry.pose.y = localization.y;
    odometry.pose.theta = Degree(localization.yaw + 90).Get<Radian>();
    odometry.v.x = chassis.speed / 3.6;


    /// 进行位置偏移矫正（临时）
    double theta = odometry.pose.theta;
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);
    double lx = 0.337;
    double ly = 0.14;

    odometry.pose.x = odometry.pose.t.x + lx * cos_theta - ly * sin_theta;
    odometry.pose.y = odometry.pose.t.y + lx * sin_theta + ly * cos_theta;

    double vx = Velocity.twist.twist.linear.x;
    double vy = Velocity.twist.twist.linear.y;
    double wz = Velocity.twist.twist.angular.z;

    odometry.v.y = vy * cos_theta - vx * sin_theta;
    odometry.w.z = wz;

    odometry.To(vehicle_state.value());
    vehicle_state.value().header.frame_id = "world";

    SendTF("world", "ego", odometry.pose);
}

