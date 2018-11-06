#include <GPS/GPS.h>
#include <GPS/.GPSModule.h>

using namespace nox::app;
USING_NAMESPACE_NOX;

void GPS::Initialize()
{
    in.SetLength(72).SetMode(StreamMailbox::FixLength);
    if(not in.Subscribe({params.GPS.Interface, "255.255.255.255", 3000}))
    {
        Logger::E("GPS") << "Fail to open gps at " << params.GPS.Interface;
        Finish();
        return;
    }
    
    
}

void GPS::Terminate()
{
    GPSModule::Terminate();
}

void GPS::Process(optional<nav_msgs::Odometry> &vehicle_state)
{
    GPSModule::Process(vehicle_state);
}
