#include <Planner/.PlannerModule.h>
#include <Planner/Planner.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(Planner);

void PlannerModule::OnStart()
{
    /// 配置节点
    SetFrequency( 20.000000 );
    Viewer::Instance()->SetRender(std::make_shared< None >());

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
    
    Ptr<type::Position> localization_ptr;
    
    if(!mailboxes.localization.IsFresh())
    {
        status = false;
        
        OnlocalizationFail(  );
        ProcessOutput(  );
    }
    else
        localization_ptr = std::make_shared<type::Position>(std::move( mailboxes.localization.Get(false) ));

    if(status)
    {
        
        Process( localization_ptr  );
        ProcessOutput(  );
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
    
    mailboxes.localization.Subscribe({"localization"});
    mailboxes.localization.SetValidity(1000);
    
}

void PlannerModule::InitParameter()
{
    
}

void PlannerModule::InitCallback()
{
    
    mailboxes.localization.AddCallback([&]( const type::Position & msg, const Address & address ) {
        
        bool result = ProcessOnlocalization( PtrIn<type::Position>(new type::Position(msg))  );
        ProcessOutput(  );
        return result;
    });

}

void PlannerModule::InitPlugin()
{
    
}

void PlannerModule::TerminateMailbox()
{
    
    mailboxes.localization.UnSubscribe();
}

void PlannerModule::TerminatePlugin()
{
    
}

void PlannerModule::ProcessOutput(  )
{
    
}

void PlannerModule::Initialize()
{
    // Do Nothing ...
}

void PlannerModule::Terminate()
{
    // Do Nothing ...
}

void PlannerModule::Process( PtrIn<type::Position> localization  )
{
    
}


void PlannerModule::OnlocalizationFail(  )
{
    
}


bool PlannerModule::ProcessOnlocalization( PtrIn<type::Position> localization   )
{
    
    return false; /// 
}
