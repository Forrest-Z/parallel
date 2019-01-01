#include <Repeater/Repeater.h>
#include <Common/Common.h>
#include <Repeater/.RepeaterModule.h>

using namespace nox::app;
USING_NAMESPACE_NOX;



void Repeater::Process(
    nox_msgs::Location localization,
    nox_msgs::Chassis chassis,
    optional<geometry_msgs::TwistWithCovarianceStamped> velocity,
    optional<nav_msgs::Odometry> &vehicle_state,
    optional<nox_msgs::Location> &location,
    double lx, double ly)
{
    //region 处理基本位置和线速度
    type::Odometry odometry;
    odometry.pose.x = localization.x;
    odometry.pose.y = localization.y;
    odometry.pose.theta = Degree(localization.yaw + 90).Get<Radian>();
    odometry.v.x = chassis.speed / 3.6;
    //endregion

    //region 进行位置偏移矫正（临时）
    double theta = odometry.pose.theta;
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);

    odometry.pose.x = odometry.pose.t.x + lx * cos_theta - ly * sin_theta;
    odometry.pose.y = odometry.pose.t.y + lx * sin_theta + ly * cos_theta;
    //endregion

    //region 更新速度
    geometry_msgs::TwistWithCovarianceStamped Velocity;
    if(velocity)
        Velocity = velocity.value();

    double vx = Velocity.twist.twist.linear.x;
    double vy = Velocity.twist.twist.linear.y;
    double wz = Velocity.twist.twist.angular.z;

    odometry.v.y = vy * cos_theta - vx * sin_theta;
    odometry.w.z = wz;
    //endregion

    //region 赋值到输出
    vehicle_state.emplace();
    location.emplace();

    odometry.To(vehicle_state.value());
    vehicle_state.value().header.frame_id = "world";

    auto & location_ = location.value();
    location_.header = localization.header;
    location_.x = odometry.pose.x;
    location_.y = odometry.pose.y;
    location_.yaw = localization.yaw;

    SendTF("world", "ego", odometry.pose);
    //endregion
}


void Repeater::Process(optional<nox_msgs::Location> gps_localization, nox_msgs::Chassis chassis,
                       optional<geometry_msgs::TwistWithCovarianceStamped> Velocity,
                       optional<nox_msgs::Location> lidar_localization, optional<nav_msgs::Odometry> &vehicle_state,
                       optional<nox_msgs::Location> &localization)
{
    if(gps_localization)
        Process(gps_localization.value(), chassis, Velocity, vehicle_state, localization, 0.337, 0.14);
    else if(lidar_localization)
        Process(lidar_localization.value(), chassis, Velocity, vehicle_state, localization, -1.631, 0);
}

