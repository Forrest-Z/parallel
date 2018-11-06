#include <Tracking/.TrackingModule.h>
#include <Tracking/Tracking.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Tracking);

void TrackingModule::OnStart()
{
    /// 配置节点
    SetFrequency( 20.000000 );
    Viewer::Instance()->SetRender(New< None >());

    // TODO： 看门狗

    /// 框架初始化
    InitParameter();
    InitPlugin();
    InitMailbox();
    InitCallback();

    /// 用户模块初始化
    Initialize();
}

void TrackingModule::OnRun()
{
    bool status = true;
    
    optional<nox_msgs::Trajectory> trajectory_in;
    nav_msgs::Odometry vehicle_state_in;
    nox_msgs::Chassis chassis_in;
    
    if(!mailboxes.trajectory.IsFresh()) {} 
    else
        trajectory_in = mailboxes.trajectory.Get();

    if(!mailboxes.vehicle_state.IsFresh())
    {
        status = false;
        
        optional<nox_msgs::DrivingCommand> driving_out;
        Onvehicle_stateFail( driving_out );
        ProcessOutput( driving_out );
    }
    else
        vehicle_state_in = mailboxes.vehicle_state.Get();

    if(!mailboxes.chassis.IsFresh())
    {
        status = false;
        
        optional<nox_msgs::DrivingCommand> driving_out;
        OnchassisFail( driving_out );
        ProcessOutput( driving_out );
    }
    else
        chassis_in = mailboxes.chassis.Get();

    if(status)
    {
        
        optional<nox_msgs::DrivingCommand> driving_out;
        Process( trajectory_in, vehicle_state_in, chassis_in,  driving_out );
        ProcessOutput( driving_out );
    }
}

void TrackingModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void TrackingModule::InitMailbox()
{
    
    mailboxes.trajectory.Subscribe({"trajectory"});
    mailboxes.trajectory.SetValidity(1000);
    mailboxes.vehicle_state.Subscribe({"vehicle_state"});
    mailboxes.vehicle_state.SetValidity(1000);
    mailboxes.chassis.Subscribe({"chassis"});
    mailboxes.chassis.SetValidity(1000);
    
    mailboxes.driving.Advertise({"driving"});
}

void TrackingModule::InitParameter()
{
    
    params.Vehicle.Read(GetParameter("Vehicle"));
    params.VTF.Read(GetParameter("VTF"));
}

void TrackingModule::InitCallback()
{
    
}

void TrackingModule::InitPlugin()
{
    
}

void TrackingModule::TerminateMailbox()
{
    
    mailboxes.trajectory.UnSubscribe();
    mailboxes.vehicle_state.UnSubscribe();
    mailboxes.chassis.UnSubscribe();
}

void TrackingModule::TerminatePlugin()
{
    
}

void TrackingModule::ProcessOutput( optional<nox_msgs::DrivingCommand> & driving )
{
    
    if(driving)
        mailboxes.driving.Send(driving.value());
}

void TrackingModule::Initialize()
{
    // Do Nothing ...
}

void TrackingModule::Terminate()
{
    // Do Nothing ...
}

void TrackingModule::Process( optional<nox_msgs::Trajectory> trajectory, nav_msgs::Odometry vehicle_state, nox_msgs::Chassis chassis,  optional<nox_msgs::DrivingCommand> & driving )
{
    
    driving.reset();
}


void TrackingModule::Onvehicle_stateFail( optional<nox_msgs::DrivingCommand> & driving )
{
    
    driving.reset();
}
void TrackingModule::OnchassisFail( optional<nox_msgs::DrivingCommand> & driving )
{
    
    driving.reset();
}


