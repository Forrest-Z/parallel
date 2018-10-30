#include <Simulator/.SimulatorModule.h>
#include <Simulator/Simulator.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Simulator);

void SimulatorModule::OnStart()
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

void SimulatorModule::OnRun()
{
    bool status = true;
    
    
    if(status)
    {
        
        optional<nav_msgs::Odometry> vehicle_state_out;
        optional<nox_msgs::Location> Localization_out;
        optional<nox_lcm::GPSData> GPSDataLCM_out;
        Process(  vehicle_state_out, Localization_out, GPSDataLCM_out );
        ProcessOutput( vehicle_state_out, Localization_out, GPSDataLCM_out );
    }
}

void SimulatorModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void SimulatorModule::InitMailbox()
{
    
    
    mailboxes.vehicle_state.Advertise({"vehicle_state"});
    mailboxes.Localization.Advertise({"Localization"});
    mailboxes.GPSDataLCM.Advertise({"GPSData"});
}

void SimulatorModule::InitParameter()
{
    
}

void SimulatorModule::InitCallback()
{
    
}

void SimulatorModule::InitPlugin()
{
    
}

void SimulatorModule::TerminateMailbox()
{
    
}

void SimulatorModule::TerminatePlugin()
{
    
}

void SimulatorModule::ProcessOutput( optional<nav_msgs::Odometry> & vehicle_state, optional<nox_msgs::Location> & Localization, optional<nox_lcm::GPSData> & GPSDataLCM )
{
    
    if(vehicle_state)
        mailboxes.vehicle_state.Send(vehicle_state.value());
    if(Localization)
        mailboxes.Localization.Send(Localization.value());
    if(GPSDataLCM)
        mailboxes.GPSDataLCM.Send(GPSDataLCM.value());
}

void SimulatorModule::Initialize()
{
    // Do Nothing ...
}

void SimulatorModule::Terminate()
{
    // Do Nothing ...
}

void SimulatorModule::Process(  optional<nav_msgs::Odometry> & vehicle_state, optional<nox_msgs::Location> & Localization, optional<nox_lcm::GPSData> & GPSDataLCM )
{
    
    vehicle_state.reset();
    Localization.reset();
    GPSDataLCM.reset();
}




