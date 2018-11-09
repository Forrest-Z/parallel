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
    
    optional<geometry_msgs::PoseWithCovarianceStamped> obstacle_in;
    optional<nav_msgs::Odometry> vehicle_state_in;
    
    if(!mailboxes.obstacle.IsFresh()) {} 
    else
        obstacle_in = mailboxes.obstacle.Get();

    if(!mailboxes.vehicle_state.IsFresh()) {} 
    else
        vehicle_state_in = mailboxes.vehicle_state.Get();

    if(status)
    {
        
        optional<nox_msgs::ObstacleArray> obstacles_out;
        Process( obstacle_in, vehicle_state_in,  obstacles_out );
        ProcessOutput( obstacles_out );
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
    
    mailboxes.obstacle.Subscribe({"initialpose"});
    mailboxes.obstacle.SetValidity(1000);
    mailboxes.vehicle_state.Subscribe({"vehicle_state"});
    mailboxes.vehicle_state.SetValidity(1000);
    
    mailboxes.obstacles.Advertise({"virtual_obstacles"});
}

void SimulatorModule::InitParameter()
{
    
}

void SimulatorModule::InitCallback()
{
    
    mailboxes.obstacle.AddCallback([&]( const geometry_msgs::PoseWithCovarianceStamped & msg, const Address & address ) {
        
        optional<nox_msgs::ObstacleArray> obstacles_out;
        bool result = ProcessOnobstacle( msg , obstacles_out );
        ProcessOutput( obstacles_out );
        return result;
    });

    mailboxes.vehicle_state.AddCallback([&]( const nav_msgs::Odometry & msg, const Address & address ) {
        
        optional<nox_msgs::ObstacleArray> obstacles_out;
        bool result = ProcessOnvehicle_state( msg , obstacles_out );
        ProcessOutput( obstacles_out );
        return result;
    });

}

void SimulatorModule::InitPlugin()
{
    
}

void SimulatorModule::TerminateMailbox()
{
    
    mailboxes.obstacle.UnSubscribe();
    mailboxes.vehicle_state.UnSubscribe();
}

void SimulatorModule::TerminatePlugin()
{
    
}

void SimulatorModule::ProcessOutput( optional<nox_msgs::ObstacleArray> & obstacles )
{
    
    if(obstacles)
        mailboxes.obstacles.Send(obstacles.value());
}

void SimulatorModule::Initialize()
{
    // Do Nothing ...
}

void SimulatorModule::Terminate()
{
    // Do Nothing ...
}

void SimulatorModule::Process( optional<geometry_msgs::PoseWithCovarianceStamped> obstacle, optional<nav_msgs::Odometry> vehicle_state,  optional<nox_msgs::ObstacleArray> & obstacles )
{
    
    obstacles.reset();
}




bool SimulatorModule::ProcessOnobstacle( geometry_msgs::PoseWithCovarianceStamped obstacle , optional<nox_msgs::ObstacleArray> & obstacles )
{
    
    obstacles.reset();
    return false; /// 
}
bool SimulatorModule::ProcessOnvehicle_state( nav_msgs::Odometry vehicle_state , optional<nox_msgs::ObstacleArray> & obstacles )
{
    
    obstacles.reset();
    return false; /// 
}