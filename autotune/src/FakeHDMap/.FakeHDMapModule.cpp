#include <FakeHDMap/.FakeHDMapModule.h>
#include <FakeHDMap/FakeHDMap.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

LaunchModule(FakeHDMap);

void FakeHDMapModule::OnStart()
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

void FakeHDMapModule::OnRun()
{
    bool status = true;
    
    
    if(status)
    {
        
        optional<std_msgs::String> hdmap_out;
        Process(  hdmap_out );
        ProcessOutput( hdmap_out );
    }
}

void FakeHDMapModule::OnFinish()
{
    Terminate();
    TerminateMailbox();
    TerminatePlugin();
}

void FakeHDMapModule::InitMailbox()
{
    
    
    mailboxes.hdmap.Advertise({"hdmap"});
}

void FakeHDMapModule::InitParameter()
{
    
}

void FakeHDMapModule::InitCallback()
{
    
}

void FakeHDMapModule::InitPlugin()
{
    
}

void FakeHDMapModule::TerminateMailbox()
{
    
}

void FakeHDMapModule::TerminatePlugin()
{
    
}

void FakeHDMapModule::ProcessOutput( optional<std_msgs::String> & hdmap )
{
    
    if(hdmap)
        mailboxes.hdmap.Send(hdmap.value());
}

void FakeHDMapModule::Initialize()
{
    // Do Nothing ...
}

void FakeHDMapModule::Terminate()
{
    // Do Nothing ...
}

void FakeHDMapModule::Process(  optional<std_msgs::String> & hdmap )
{
    
    hdmap.reset();
}




