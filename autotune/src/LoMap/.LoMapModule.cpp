#include <LoMap/.LoMapModule.h>
#include <LoMap/LoMap.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(LoMap);

void LoMapModule::OnStart()
{
    /// 配置节点
    SetFrequency( 20.000000 );
    Viewer::Instance()->SetRender(New< Rviz >());

    // TODO： 看门狗

    /// 框架初始化
    InitParameter();
    InitPlugin();
    InitMailbox();
    InitCallback();

    /// 用户模块初始化
    Initialize();
}

void LoMapModule::OnRun()
{
    bool status = true;
    
    optional<nav_msgs::Odometry> vehicle_state_in;
    optional<nox_msgs::ObstacleArray> obstacles_in;
    optional<nox_msgs::Road> old_map_in;
    optional<std_msgs::String> hdmap_in;
    optional<nox_msgs::ObstacleArray> virtual_obstacles_in;
    optional<traffic_light::msg_traffic_light_list> traffic_lights_in;
    
    if(!mailboxes.vehicle_state.IsFresh()) {} 
    else
        vehicle_state_in = mailboxes.vehicle_state.Get();

    if(!mailboxes.obstacles.IsFresh()) {} 
    else
        obstacles_in = mailboxes.obstacles.Get();

    if(!mailboxes.old_map.IsFresh()) {} 
    else
        old_map_in = mailboxes.old_map.Get();

    if(!mailboxes.hdmap.IsFresh()) {} 
    else
        hdmap_in = mailboxes.hdmap.Get();

    if(!mailboxes.virtual_obstacles.IsFresh()) {} 
    else
        virtual_obstacles_in = mailboxes.virtual_obstacles.Get();

    if(!mailboxes.traffic_lights.IsFresh()) {} 
    else
        traffic_lights_in = mailboxes.traffic_lights.Get();

    if(status)
    {
        
        Process( vehicle_state_in, obstacles_in, old_map_in, hdmap_in, virtual_obstacles_in, traffic_lights_in  );
        ProcessOutput(  );
    }
}

void LoMapModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void LoMapModule::InitMailbox()
{
    
    mailboxes.vehicle_state.Subscribe({"vehicle_state"});
    mailboxes.vehicle_state.SetValidity(1000);
    mailboxes.obstacles.Subscribe({"obstacles"});
    mailboxes.obstacles.SetValidity(1000);
    mailboxes.old_map.Subscribe({"RoadPlanning"});
    mailboxes.old_map.SetValidity(1000);
    mailboxes.hdmap.Subscribe({"map_to_planner"});
    mailboxes.hdmap.SetValidity(1000);
    mailboxes.virtual_obstacles.Subscribe({"virtual_obstacles"});
    mailboxes.virtual_obstacles.SetValidity(1000);
    mailboxes.traffic_lights.Subscribe({"traffic_lights"});
    mailboxes.traffic_lights.SetValidity(1000);
    
}

void LoMapModule::InitParameter()
{
    
}

void LoMapModule::InitCallback()
{
    
    mailboxes.vehicle_state.AddCallback([&]( const nav_msgs::Odometry & msg, const Address & address ) {
        
        bool result = ProcessOnvehicle_state( msg  );
        ProcessOutput(  );
        return result;
    });

    mailboxes.obstacles.AddCallback([&]( const nox_msgs::ObstacleArray & msg, const Address & address ) {
        
        bool result = ProcessOnobstacles( msg  );
        ProcessOutput(  );
        return result;
    });

    mailboxes.old_map.AddCallback([&]( const nox_msgs::Road & msg, const Address & address ) {
        
        bool result = ProcessOnold_map( msg  );
        ProcessOutput(  );
        return result;
    });

    mailboxes.hdmap.AddCallback([&]( const std_msgs::String & msg, const Address & address ) {
        
        bool result = ProcessOnhdmap( msg  );
        ProcessOutput(  );
        return result;
    });

    mailboxes.virtual_obstacles.AddCallback([&]( const nox_msgs::ObstacleArray & msg, const Address & address ) {
        
        bool result = ProcessOnvirtual_obstacles( msg  );
        ProcessOutput(  );
        return result;
    });

    mailboxes.traffic_lights.AddCallback([&]( const traffic_light::msg_traffic_light_list & msg, const Address & address ) {
        
        bool result = ProcessOntraffic_lights( msg  );
        ProcessOutput(  );
        return result;
    });

}

void LoMapModule::InitPlugin()
{
    
}

void LoMapModule::TerminateMailbox()
{
    
    mailboxes.vehicle_state.UnSubscribe();
    mailboxes.obstacles.UnSubscribe();
    mailboxes.old_map.UnSubscribe();
    mailboxes.hdmap.UnSubscribe();
    mailboxes.virtual_obstacles.UnSubscribe();
    mailboxes.traffic_lights.UnSubscribe();
}

void LoMapModule::TerminatePlugin()
{
    
}

void LoMapModule::ProcessOutput(  )
{
    
}

void LoMapModule::Initialize()
{
    // Do Nothing ...
}

void LoMapModule::Terminate()
{
    // Do Nothing ...
}

void LoMapModule::Process( optional<nav_msgs::Odometry> vehicle_state, optional<nox_msgs::ObstacleArray> obstacles, optional<nox_msgs::Road> old_map, optional<std_msgs::String> hdmap, optional<nox_msgs::ObstacleArray> virtual_obstacles, optional<traffic_light::msg_traffic_light_list> traffic_lights  )
{
    
}




bool LoMapModule::ProcessOnvehicle_state( nav_msgs::Odometry vehicle_state   )
{
    
    return false; /// 
}
bool LoMapModule::ProcessOnobstacles( nox_msgs::ObstacleArray obstacles   )
{
    
    return false; /// 
}
bool LoMapModule::ProcessOnold_map( nox_msgs::Road old_map   )
{
    
    return false; /// 
}
bool LoMapModule::ProcessOnhdmap( std_msgs::String hdmap   )
{
    
    return false; /// 
}
bool LoMapModule::ProcessOnvirtual_obstacles( nox_msgs::ObstacleArray virtual_obstacles   )
{
    
    return false; /// 
}
bool LoMapModule::ProcessOntraffic_lights( traffic_light::msg_traffic_light_list traffic_lights   )
{
    
    return false; /// 
}
