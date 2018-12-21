#include <Tracking/Tracking.h>
#include <Tracking/component/SimpleLongitudinalController.h>
#include <Tracking/component/VTF.h>
#include <Tracking/.TrackingModule.h>
#include <Tracking/rule/Clipping.h>
#include <Tracking/rule/Lowpass.h>
#include <Tracking/rule/Limiting.h>
#include <Tracking/rule/Custom.h>
#include "../../../.param/template/Parameter.h"
#include <Tracking/component/PCPID.h>

using namespace nox::app;
USING_NAMESPACE_NOX;
using namespace std;

void Tracking::Initialize()
{
    cache::WriteVehicleParameter(params.Vehicle);
    cache::WriteVTFParameter(params.VTF);
    cache::WritePCPIDParameter(params.PCPID);

    _lon_controller = New<SimpleLongitudinalController>();
    _lat_controller = New<PCPID>();

    _lat_controller->Initialize();

    _lon_filter.AddRule<rule::Lowpass>(5);
    _lon_filter.AddRule<rule::Clipping>(GetPeriod(), 8);
    _lon_filter.AddRule<rule::Limiting>(params.Vehicle.Steering.Min, params.Vehicle.Steering.Max);

    _timer.Start();


    Logger::I("Tracking")
        << "Vehicle Parameter: " << endl
        << string(params.Vehicle) << endl
        << "VTF Parameter: " << endl
        << string(params.VTF) << endl
        << "PCPID Parameter: " << endl
        << string(params.PCPID) << endl;
}

void Tracking::Process(optional<nox_msgs::Trajectory> trajectory, nav_msgs::Odometry vehicle_state,
                       nox_msgs::Chassis chassis, optional<nox_msgs::DrivingCommand> &driving)
{
    /// 1. 初始化车的状态
    tool::Console::Screen::Clear();

    if(trajectory)
        _trajectory.From(trajectory.value());

    _vehicle.From(vehicle_state);
    _vehicle.From(chassis);

    /// 2. 使用横纵向控制器计算输出速度与转角
    Logger::I("Tracking").Print("Vehicle(t, x, y, theta, v, w): %6.2lf s, %8.4lf m, %8.4lf m, %8.4lf deg, %6.2lf km/h, %6.2lf deg",
        _timer.Watch().Get<Second>(),
        _vehicle.pose.t.x,
        _vehicle.pose.t.y,
        _vehicle.pose.theta * 180.0 / M_PI,
        chassis.speed,
        chassis.steering);

    double steering = 0;
    double speed = 0;
    
    if(not _trajectory.Empty())
    {
        speed    = _lon_controller->Calculate(_trajectory, _vehicle);
        steering = _lat_controller->Calculate(_trajectory, _vehicle);
    }

    /// 3. 处理计算结果（单位转化、滤波）
    speed    *= 3.6;
    steering *= 180.0 / M_PI * params.Vehicle.Steering.Ratio + params.Vehicle.Steering.Offset;
    steering  = _lon_filter(steering);
    Logger::I("Tracking").Print("Result(v, w): %6.2lf km/h, %6.2lf deg", speed, steering);

    /// 4. 输出结果
    driving.emplace();
    Clock::now().To(driving.value().header);
    driving.value().target_speed    = speed;
    driving.value().target_steering = steering;
}

