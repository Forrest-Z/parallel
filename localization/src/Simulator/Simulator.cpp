#include <Simulator/Simulator.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

void Simulator::Initialize()
{
    static ControlPanel panel;
    panel.Add({KeyBoard::A, KeyBoard::a}, "turn left", [&](){

        _odometry.pose.theta = (Radian(_odometry.pose.theta) + Degree(1)).Get(Angle::Radian);

    }).Add({KeyBoard::d, KeyBoard::D}, "turn right", [&](){

        _odometry.pose.theta = (Radian(_odometry.pose.theta) - Degree(1)).Get(Angle::Radian);

    }).Add({KeyBoard::w, KeyBoard::W}, "move forward", [&](){

        _odometry.pose = _odometry.pose.Move(1);

    }).Add({KeyBoard::S, KeyBoard::s}, "move backward", [&](){

        _odometry.pose = _odometry.pose.Move(-1);

    }).PrintHelp();

}


void Simulator::Process(optional<nav_msgs::Odometry> &vehicle_state, optional<nox_msgs::Location> &Localization)
{
    vehicle_state.emplace();
    vehicle_state.value().header.frame_id = "nox";
    _odometry.To(vehicle_state.value());

    Localization.emplace();
    Localization.value().x = _odometry.pose.x;
    Localization.value().y = _odometry.pose.y;
    Localization.value().yaw = (Radian(_odometry.pose.theta) + Degree(270)).Get(Angle::Degree);
}
