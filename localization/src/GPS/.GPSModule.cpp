#include <GPS/.GPSModule.h>
#include <GPS/GPS.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(GPS);

void GPSModule::OnStart()
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

void GPSModule::OnRun()
{
    bool status = true;
    
    
    if(status)
    {
        
        optional<nav_msgs::Odometry> vehicle_state_out;
        Process(  vehicle_state_out );
        ProcessOutput( vehicle_state_out );
    }
}

void GPSModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void GPSModule::InitMailbox()
{
    
    
    mailboxes.vehicle_state.Advertise({"vehicle_state"});
}

void GPSModule::InitParameter()
{
    
    params.GPS.Read(GetParameter("GPS"));
}

void GPSModule::InitCallback()
{
    
}

void GPSModule::InitPlugin()
{
    
}

void GPSModule::TerminateMailbox()
{
    
}

void GPSModule::TerminatePlugin()
{
    
}

void GPSModule::ProcessOutput( optional<nav_msgs::Odometry> & vehicle_state )
{
    
    if(vehicle_state)
        mailboxes.vehicle_state.Send(vehicle_state.value());
}

void GPSModule::Initialize()
{
    // Do Nothing ...
}

void GPSModule::Terminate()
{
    // Do Nothing ...
}

void GPSModule::Process(  optional<nav_msgs::Odometry> & vehicle_state )
{
    
    vehicle_state.reset();
}




