#include <SimpleMap/.SimpleMapModule.h>
#include <SimpleMap/SimpleMap.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(SimpleMap);

void SimpleMapModule::OnStart()
{
    /// 配置节点
    SetFrequency( 20.000000 );
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

void SimpleMapModule::OnRun()
{
    bool status = true;
    
    if(not mailboxes.vehicle_state.IsFresh())
    {
        status = false;
        
        optional<nox_msgs::Trajectory> simple_map_out;
        Onvehicle_stateFail( simple_map_out );
        ProcessOutput( simple_map_out );
    }

    if(status)
    {
        
        optional<nox_msgs::Trajectory> simple_map_out;
        Process( 
            
            mailboxes.vehicle_state.Get(), 
            simple_map_out 
        );
        ProcessOutput( simple_map_out );
    }
}

void SimpleMapModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void SimpleMapModule::InitMailbox()
{
    
    mailboxes.vehicle_state.Subscribe({"vehicle_state"});
    mailboxes.vehicle_state.SetValidity(Millisecond(1000));
    
    mailboxes.simple_map.Advertise({"simple_map"});
}

void SimpleMapModule::InitParameter()
{
    
}

void SimpleMapModule::InitCallback()
{
    
}

void SimpleMapModule::InitPlugin()
{
    
}

void SimpleMapModule::TerminateMailbox()
{
    
    mailboxes.vehicle_state.UnSubscribe();
}

void SimpleMapModule::TerminatePlugin()
{
    
}

void SimpleMapModule::ProcessOutput( optional<nox_msgs::Trajectory> & simple_map )
{
    
    if(simple_map)
        mailboxes.simple_map.Send(simple_map.value());
}

void SimpleMapModule::Initialize()
{
    // Do Nothing ...
}

void SimpleMapModule::Terminate()
{
    // Do Nothing ...
}

void SimpleMapModule::Process( nav_msgs::Odometry vehicle_state,  optional<nox_msgs::Trajectory> & simple_map )
{
    
    simple_map.reset();
}


void SimpleMapModule::Onvehicle_stateFail( optional<nox_msgs::Trajectory> & simple_map )
{
    
    simple_map.reset();
}


