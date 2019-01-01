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
    

    if(status)
    {
        
        optional<nox_msgs::ObstacleArray> obstacles_out;
        Process( 
            
            mailboxes.jsk_obstacles.IsFresh() ? mailboxes.jsk_obstacles.Get() : optional<autoware_msgs::DetectedObjectArray>(), 
            obstacles_out 
        );
        ProcessOutput( obstacles_out );
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
    
    mailboxes.jsk_obstacles.Subscribe({"detected_objects"});
    mailboxes.jsk_obstacles.SetValidity(Millisecond(1000));
    
    mailboxes.obstacles.Advertise({"obstacles"});
}

void RepeaterModule::InitParameter()
{
    
}

void RepeaterModule::InitCallback()
{
    
    mailboxes.jsk_obstacles.AddCallback([&]( const autoware_msgs::DetectedObjectArray & msg, const Address & address ) {
        
        optional<nox_msgs::ObstacleArray> obstacles_out;
        bool result = ProcessOnjsk_obstacles( msg , obstacles_out );
        ProcessOutput( obstacles_out );
        return result;
    });

}

void RepeaterModule::InitPlugin()
{
    
}

void RepeaterModule::TerminateMailbox()
{
    
    mailboxes.jsk_obstacles.UnSubscribe();
}

void RepeaterModule::TerminatePlugin()
{
    
}

void RepeaterModule::ProcessOutput( optional<nox_msgs::ObstacleArray> & obstacles )
{
    
    if(obstacles)
        mailboxes.obstacles.Send(obstacles.value());
}

void RepeaterModule::Initialize()
{
    // Do Nothing ...
}

void RepeaterModule::Terminate()
{
    // Do Nothing ...
}

void RepeaterModule::Process( optional<autoware_msgs::DetectedObjectArray> jsk_obstacles,  optional<nox_msgs::ObstacleArray> & obstacles )
{
    
    obstacles.reset();
}




bool RepeaterModule::ProcessOnjsk_obstacles( autoware_msgs::DetectedObjectArray jsk_obstacles , optional<nox_msgs::ObstacleArray> & obstacles )
{
    
    obstacles.reset();
    return false; /// 
}
