#include <Repeater/.RepeaterModule.h>
#include <Repeater/Repeater.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Repeater);

void RepeaterModule::OnStart()
{
    /// 配置节点
    SetFrequency( 100.000000 );
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

void RepeaterModule::OnRun()
{
    bool status = true;
    

    if(not mailboxes.chassis.IsFresh())
    {
        status = false;
        
        optional<nav_msgs::Odometry> vehicle_state_out;
        optional<nox_msgs::Location> localization_out;
        OnchassisFail( vehicle_state_out, localization_out );
        ProcessOutput( vehicle_state_out, localization_out );
    }



    if(status)
    {
        
        optional<nav_msgs::Odometry> vehicle_state_out;
        optional<nox_msgs::Location> localization_out;
        Process( 
            
            mailboxes.gps_localization.IsFresh() ? mailboxes.gps_localization.Get() : optional<nox_msgs::Location>(),
            mailboxes.chassis.Get(),
            mailboxes.Velocity.IsFresh() ? mailboxes.Velocity.Get() : optional<geometry_msgs::TwistWithCovarianceStamped>(),
            mailboxes.lidar_localization.IsFresh() ? mailboxes.lidar_localization.Get() : optional<nox_msgs::Location>(), 
            vehicle_state_out, localization_out 
        );
        ProcessOutput( vehicle_state_out, localization_out );
    }
}

void RepeaterModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void RepeaterModule::InitMailbox()
{
    
    mailboxes.gps_localization.Subscribe({"gps/Localization"});
    mailboxes.gps_localization.SetValidity(Millisecond(1000));
    mailboxes.chassis.Subscribe({"chassis"});
    mailboxes.chassis.SetValidity(Millisecond(1000));
    mailboxes.Velocity.Subscribe({"/gps/vel"});
    mailboxes.Velocity.SetValidity(Millisecond(1000));
    mailboxes.lidar_localization.Subscribe({"lidar/Localization"});
    mailboxes.lidar_localization.SetValidity(Millisecond(1000));
    
    mailboxes.vehicle_state.Advertise({"vehicle_state"});
    mailboxes.localization.Advertise({"Localization"});
}

void RepeaterModule::InitParameter()
{
    
}

void RepeaterModule::InitCallback()
{
    
}

void RepeaterModule::InitPlugin()
{
    
}

void RepeaterModule::TerminateMailbox()
{
    
    mailboxes.gps_localization.UnSubscribe();
    mailboxes.chassis.UnSubscribe();
    mailboxes.Velocity.UnSubscribe();
    mailboxes.lidar_localization.UnSubscribe();
}

void RepeaterModule::TerminatePlugin()
{
    
}

void RepeaterModule::ProcessOutput( optional<nav_msgs::Odometry> & vehicle_state, optional<nox_msgs::Location> & localization )
{
    
    if(vehicle_state)
        mailboxes.vehicle_state.Send(vehicle_state.value());
    if(localization)
        mailboxes.localization.Send(localization.value());
}

void RepeaterModule::Initialize()
{
    // Do Nothing ...
}

void RepeaterModule::Terminate()
{
    // Do Nothing ...
}

void RepeaterModule::Process( optional<nox_msgs::Location> gps_localization, nox_msgs::Chassis chassis, optional<geometry_msgs::TwistWithCovarianceStamped> Velocity, optional<nox_msgs::Location> lidar_localization,  optional<nav_msgs::Odometry> & vehicle_state, optional<nox_msgs::Location> & localization )
{
    
    vehicle_state.reset();
    localization.reset();
}


void RepeaterModule::OnchassisFail( optional<nav_msgs::Odometry> & vehicle_state, optional<nox_msgs::Location> & localization )
{
    
    vehicle_state.reset();
    localization.reset();
}


