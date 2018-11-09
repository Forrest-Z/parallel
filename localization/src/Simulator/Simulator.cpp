#include <Simulator/Simulator.h>
#include <Simulator/tool/CoordinateConverter.h>
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

        _odometry.pose = _odometry.pose.Move(1.0);

    }).Add({KeyBoard::S, KeyBoard::s}, "move backward", [&](){

        _odometry.pose = _odometry.pose.Move(-1.0);

    }).Add({KeyBoard::Q, KeyBoard::q}, "add speed", [&](){

        ++_odometry.v.x;

    }).Add({KeyBoard::e, KeyBoard::E}, "reduce speed", [&](){

        if(--_odometry.v.x < 0)
            _odometry.v.x = 0;

    }).PrintHelp();

    CoordinateConverter::Setting(120.7752, 31.5921);
}

void Simulator::Process(optional<nav_msgs::Odometry> &vehicle_state, optional<nox_msgs::Location> &Localization,
                        optional<nox_lcm::GPSData> &GPSDataLCM, optional<nox_msgs::Chassis> &chassis,
                        optional<nox_msgs::Location> &localization)
{
    vehicle_state.emplace();
    vehicle_state.value().header.frame_id = "nox";
    _odometry.To(vehicle_state.value());

    Localization.emplace();
    Localization.value().x = _odometry.pose.x;
    Localization.value().y = _odometry.pose.y;
    Localization.value().yaw = (Radian(_odometry.pose.theta) + Degree(270)).Get(Angle::Degree);

    GPSDataLCM.emplace();
    CoordinateConverter::xy2ll(Localization.value().x, Localization.value().y,
                               GPSDataLCM.value().longitude, GPSDataLCM.value().latitude);
    GPSDataLCM.value().heading = Degree(-Localization.value().yaw).Get();
    GPSDataLCM.value().time = Clock::ms();

    chassis.emplace();
    chassis.value().speed = _odometry.v.x;

    localization.emplace(Localization.value());
}
