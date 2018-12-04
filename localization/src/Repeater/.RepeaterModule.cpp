#include <Repeater/.RepeaterModule.h>
#include <Repeater/Repeater.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Repeater);

void RepeaterModule::OnStart()
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

void RepeaterModule::OnRun()
{
    bool status = true;
    
    nox_msgs::Location localization_in;
    nox_msgs::Chassis chassis_in;
    
    if(!mailboxes.localization.IsFresh())
    {
        status = false;
        
        optional<nav_msgs::Odometry> vehicle_state_out;
        OnlocalizationFail( vehicle_state_out );
        ProcessOutput( vehicle_state_out );
    }
    else
        localization_in = mailboxes.localization.Get();

    if(!mailboxes.chassis.IsFresh())
    {
        status = false;
        
        optional<nav_msgs::Odometry> vehicle_state_out;
        OnchassisFail( vehicle_state_out );
        ProcessOutput( vehicle_state_out );
    }
    else
        chassis_in = mailboxes.chassis.Get();

    if(status)
    {
        
        optional<nav_msgs::Odometry> vehicle_state_out;
        Process( localization_in, chassis_in,  vehicle_state_out );
        ProcessOutput( vehicle_state_out );
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
    
    mailboxes.localization.Subscribe({"/gps/Localization"});
    mailboxes.localization.SetValidity(1000);
    mailboxes.chassis.Subscribe({"chassis"});
    mailboxes.chassis.SetValidity(1000);
    
    mailboxes.vehicle_state.Advertise({"vehicle_state"});
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
    
    mailboxes.localization.UnSubscribe();
    mailboxes.chassis.UnSubscribe();
}

void RepeaterModule::TerminatePlugin()
{
    
}

void RepeaterModule::ProcessOutput( optional<nav_msgs::Odometry> & vehicle_state )
{
    
    if(vehicle_state)
        mailboxes.vehicle_state.Send(vehicle_state.value());
}

void RepeaterModule::Initialize()
{
    // Do Nothing ...
}

void RepeaterModule::Terminate()
{
    // Do Nothing ...
}

void RepeaterModule::Process( nox_msgs::Location localization, nox_msgs::Chassis chassis,  optional<nav_msgs::Odometry> & vehicle_state )
{
    
    vehicle_state.reset();
}


void RepeaterModule::OnlocalizationFail( optional<nav_msgs::Odometry> & vehicle_state )
{
    
    vehicle_state.reset();
}
void RepeaterModule::OnchassisFail( optional<nav_msgs::Odometry> & vehicle_state )
{
    
    vehicle_state.reset();
}


