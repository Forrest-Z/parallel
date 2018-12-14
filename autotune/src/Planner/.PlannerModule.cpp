#include <Planner/.PlannerModule.h>
#include <Planner/Planner.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Planner);

void PlannerModule::OnStart()
{
    /// 配置节点
    SetFrequency( 20.000000 );
    Viewer::Instance()->SetRender(New< Rviz >());
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

void PlannerModule::OnRun()
{
    bool status = true;
    
    nav_msgs::Odometry vehicle_state_in;
    
    if(!mailboxes.vehicle_state.IsFresh())
    {
        status = false;
        
        optional<nox_msgs::Trajectory> trajectory_out;
        Onvehicle_stateFail( trajectory_out );
        ProcessOutput( trajectory_out );
    }
    else
        vehicle_state_in = mailboxes.vehicle_state.Get();

    if(status)
    {
        
        optional<nox_msgs::Trajectory> trajectory_out;
        Process( vehicle_state_in,  trajectory_out );
        ProcessOutput( trajectory_out );
    }
}

void PlannerModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void PlannerModule::InitMailbox()
{
    
    mailboxes.vehicle_state.Subscribe({"vehicle_state"});
    mailboxes.vehicle_state.SetValidity(1000);
    
    mailboxes.trajectory.Advertise({"trajectory"});
}

void PlannerModule::InitParameter()
{
    
    params.Vehicle.Read(GetParameter("Vehicle"));
}

void PlannerModule::InitCallback()
{
    
}

void PlannerModule::InitPlugin()
{
    
}

void PlannerModule::TerminateMailbox()
{
    
    mailboxes.vehicle_state.UnSubscribe();
}

void PlannerModule::TerminatePlugin()
{
    
}

void PlannerModule::ProcessOutput( optional<nox_msgs::Trajectory> & trajectory )
{
    
    if(trajectory)
        mailboxes.trajectory.Send(trajectory.value());
}

void PlannerModule::Initialize()
{
    // Do Nothing ...
}

void PlannerModule::Terminate()
{
    // Do Nothing ...
}

void PlannerModule::Process( nav_msgs::Odometry vehicle_state,  optional<nox_msgs::Trajectory> & trajectory )
{
    
    trajectory.reset();
}


void PlannerModule::Onvehicle_stateFail( optional<nox_msgs::Trajectory> & trajectory )
{
    
    trajectory.reset();
}


