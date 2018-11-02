#include <Tracking/.TrackingModule.h>
#include <Tracking/Tracking.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Tracking);

void TrackingModule::OnStart()
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

void TrackingModule::OnRun()
{
    bool status = true;
    
    nox_msgs::Trajectory trajectory_in;
    nav_msgs::Odometry vehicle_state_in;
    
    if(!mailboxes.trajectory.IsFresh())
    {
        status = false;
        
        OntrajectoryFail(  );
        ProcessOutput(  );
    }
    else
        trajectory_in = mailboxes.trajectory.Get();

    if(!mailboxes.vehicle_state.IsFresh())
    {
        status = false;
        
        Onvehicle_stateFail(  );
        ProcessOutput(  );
    }
    else
        vehicle_state_in = mailboxes.vehicle_state.Get();

    if(status)
    {
        
        Process( trajectory_in, vehicle_state_in  );
        ProcessOutput(  );
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
    mailboxes.trajectory.SetValidity(1000);
    mailboxes.vehicle_state.Subscribe({"vehicle_state"});
    mailboxes.vehicle_state.SetValidity(1000);
    
}

void TrackingModule::InitParameter()
{
    
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
}

void TrackingModule::TerminatePlugin()
{
    
}

void TrackingModule::ProcessOutput(  )
{
    
}

void TrackingModule::Initialize()
{
    // Do Nothing ...
}

void TrackingModule::Terminate()
{
    // Do Nothing ...
}

void TrackingModule::Process( nox_msgs::Trajectory trajectory, nav_msgs::Odometry vehicle_state  )
{
    
}


void TrackingModule::OntrajectoryFail(  )
{
    
}
void TrackingModule::Onvehicle_stateFail(  )
{
    
}


