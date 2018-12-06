#include <Simulator/Simulator.h>
#include <Simulator/tool/CoordinateConverter.h>
#include <iostream>
#include <Common/Common.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

void Simulator::Initialize()
{
    static ControlPanel panel;

    static auto PrintInfo = [&]()
    {
        Console::Screen::Clear();
        panel.PrintHelp();

        Console::ForeColor::Cyan();
        Console::Screen::Bold();

        printf("(x, y, theta, v, steering): %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf\n",
            _vehicle.pose.t.x,
            _vehicle.pose.t.y,
            _vehicle.pose.theta.get(),
            _vehicle.v.x,
            _vehicle.steering);

        Console::ForeColor::Yellow();

        printf(R"(
- - - - - - - - - - - - - - - - - - - - - - - - - -
[-] Positioning: %s
[-] Driving    : %s
- - - - - - - - - - - - - - - - - - - - - - - - - -
)", _positioning ? "ON" : "OFF",
    _driving     ? "ON" : "OFF");

        fflush(stdout);
        Console::Screen::ClearStyle();
    };

    panel.Add({KeyBoard::A, KeyBoard::a}, "turn left", [&](){

        _vehicle.pose.theta = (Radian(_vehicle.pose.theta) + Degree(1)).Get(Angle::Radian);
        _vehicle.steering = std::min(540.0, _vehicle.steering + 5);
        PrintInfo();

    }).Add({KeyBoard::d, KeyBoard::D}, "turn right", [&](){

        _vehicle.pose.theta = (Radian(_vehicle.pose.theta) - Degree(1)).Get(Angle::Radian);
        _vehicle.steering = std::max(-540.0, _vehicle.steering - 5);
        PrintInfo();

    }).Add({KeyBoard::w, KeyBoard::W}, "move forward", [&](){

        _vehicle.pose = _vehicle.pose.Move(1.0);
        PrintInfo();

    }).Add({KeyBoard::S, KeyBoard::s}, "move backward", [&](){

        _vehicle.pose = _vehicle.pose.Move(-1.0);
        PrintInfo();

    }).Add({KeyBoard::e, KeyBoard::E}, "add speed", [&](){

        ++_vehicle.v.x;
        PrintInfo();

    }).Add({KeyBoard::Q, KeyBoard::q}, "reduce speed", [&](){

        if(--_vehicle.v.x < 0)
            _vehicle.v.x = 0;
        PrintInfo();

    }).Add({KeyBoard::G, KeyBoard::g}, "enable driving", [&](){

        _driving = !_driving;
        PrintInfo();

    }).Add({KeyBoard::L, KeyBoard::l}, "enable positionting", [&](){

        _positioning = !_positioning;
        PrintInfo();

    });

    PrintInfo();

    CoordinateConverter::Setting(120.7752, 31.5921);
}

void Simulator::Process(optional<nav_msgs::Odometry> &vehicle_state, optional<nox_msgs::Location> &Localization,
                        optional<nox_lcm::GPSData> &GPSDataLCM, optional<nox_msgs::Chassis> &chassis,
                        optional<nox_msgs::Location> &localization, optional<nox_msgs::DrivingCommand> &driving)
{
    if(_positioning)
    {
        vehicle_state.emplace();
        vehicle_state.value().header.frame_id = "world";
        _vehicle.To(vehicle_state.value());

        Localization.emplace();
        Localization.value().x = _vehicle.pose.x;
        Localization.value().y = _vehicle.pose.y;
        Localization.value().yaw = (Radian(_vehicle.pose.theta) + Degree(270)).Get(Angle::Degree);

        GPSDataLCM.emplace();
        CoordinateConverter::xy2ll(Localization.value().x, Localization.value().y,
                                   GPSDataLCM.value().longitude, GPSDataLCM.value().latitude);
        GPSDataLCM.value().heading = Degree(-Localization.value().yaw).Get();
        GPSDataLCM.value().time = Clock::ms();

        chassis.emplace();
        chassis.value().speed = _vehicle.v.x;

        localization.emplace(Localization.value());

        SendTF("world", "ego", _vehicle.pose);
    }

    if(_driving)
    {
        driving.emplace();
        driving.value().target_speed = _vehicle.v.x * 3.6;
        driving.value().target_steering = _vehicle.steering;
    }
}
