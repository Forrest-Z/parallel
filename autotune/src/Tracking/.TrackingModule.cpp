#include <Tracking/.TrackingModule.h>
#include <Tracking/Tracking.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Tracking);

void TrackingModule::OnStart()
{
    /// 配置节点
    SetFrequency( 40.000000 );
    Viewer::Instance()->SetRender(New< None >());
    Viewer::Instance()->SetName(Module::GetNodeName());

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
    

    if(not mailboxes.vehicle_state.IsFresh())
    {
        status = false;
        
        optional<nox_msgs::DrivingCommand> driving_out;
        Onvehicle_stateFail( driving_out );
        ProcessOutput( driving_out );
    }

    if(not mailboxes.chassis.IsFresh())
    {
        status = false;
        
        optional<nox_msgs::DrivingCommand> driving_out;
        OnchassisFail( driving_out );
        ProcessOutput( driving_out );
    }

    if(status)
    {
        
        optional<nox_msgs::DrivingCommand> driving_out;
        Process( 
            
            mailboxes.trajectory.IsFresh() ? mailboxes.trajectory.Get() : optional<nox_msgs::Trajectory>(),
            mailboxes.vehicle_state.Get(),
            mailboxes.chassis.Get(), 
            driving_out 
        );
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
    mailboxes.trajectory.SetValidity(Millisecond(1000));
    mailboxes.vehicle_state.Subscribe({"vehicle_state"});
    mailboxes.vehicle_state.SetValidity(Millisecond(1000));
    mailboxes.chassis.Subscribe({"chassis"});
    mailboxes.chassis.SetValidity(Millisecond(1000));
    
    mailboxes.driving.Advertise({"driving"});
}

void TrackingModule::InitParameter()
{
    
    params.Vehicle.Read(GetParameter("Vehicle"));
    params.VTF.Read(GetParameter("VTF"));
    params.PCPID.Read(GetParameter("PCPID"));
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


