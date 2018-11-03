#include <Tracking/Tracking.h>
#include <Tracking/tool/SimpleLongitudinalController.h>
#include <Tracking/tool/VTF.h>
#include <Tracking/.TrackingModule.h>
#include "../../../.param/template/Parameter.h"

using namespace nox::app;
USING_NAMESPACE_NOX;

void Tracking::Initialize()
{
    _lon_controller = New<SimpleLongitudinalController>();
    _lat_controller = New<VTF>();
}

void Tracking::Process(optional<nox_msgs::Trajectory> trajectory, nav_msgs::Odometry vehicle_state,
                       nox_msgs::Chassis chassis, optional<nox_msgs::DrivingCommand> &driving)
{
    if(trajectory)
        _trajectory.From(trajectory.value());
    
    _vehicle.From(vehicle_state);
    _vehicle.From(chassis);

    double steering = 0;
    double speed = 0;
    
    if(not _trajectory.Empty())
    {
        steering = _lon_controller->Calculate(_trajectory, _vehicle);
        speed    = _lat_controller->Calculate(_trajectory, _vehicle);
    }
    
    driving.emplace();
    Clock::now().To(driving.value().header);
    driving.value().target_speed = speed * 3.6;
    driving.value().target_steering = steering * params.Vehicle.Steering.Offset;
}

