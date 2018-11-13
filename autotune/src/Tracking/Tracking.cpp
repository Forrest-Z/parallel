#include <Tracking/Tracking.h>
#include <Tracking/tool/SimpleLongitudinalController.h>
#include <Tracking/tool/VTF.h>
#include <Tracking/.TrackingModule.h>
#include "../../../.param/template/Parameter.h"

using namespace nox::app;
USING_NAMESPACE_NOX;
using namespace std;

void Tracking::Initialize()
{
    _lon_controller = New<SimpleLongitudinalController>();
    _lat_controller = New<VTF>();
    _timer.Start();

    cache::WriteVehicleParameter(params.Vehicle);
    cache::WriteVTFParameter(params.VTF);

    Logger::I("Tracking")
        << "Vehicle Parameter: " << endl
        << string(params.Vehicle) << endl
        << "VTF Parameter: " << endl
        << string(params.VTF) << endl;
}

void Tracking::Process(optional<nox_msgs::Trajectory> trajectory, nav_msgs::Odometry vehicle_state,
                       nox_msgs::Chassis chassis, optional<nox_msgs::DrivingCommand> &driving)
{
    tool::Console::Screen::Clear();

    if(trajectory)
        _trajectory.From(trajectory.value());

    _vehicle.From(vehicle_state);
    _vehicle.From(chassis);

    Logger::I("Tracking").Print("Vehicle(t, v, w): %6.2lf s, %6.2lf km/h, %6.2lf deg", _timer.Watch().Get<Second>(), chassis.speed, chassis.steering);

    double steering = 0;
    double speed = 0;
    
    if(not _trajectory.Empty())
    {
        speed    = _lon_controller->Calculate(_trajectory, _vehicle);
        steering = _lat_controller->Calculate(_trajectory, _vehicle);
    }
    
    driving.emplace();
    Clock::now().To(driving.value().header);
    driving.value().target_speed = speed * 3.6;
    driving.value().target_steering = steering * 180.0 / M_PI * params.Vehicle.Steering.Ratio + params.Vehicle.Steering.Offset;

    Logger::I("Tracking").Print("Result(v, w): %6.2lf km/h, %6.2lf deg", driving.value().target_speed, driving.value().target_steering);
}

