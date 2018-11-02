#include <FakeHDMap/FakeHDMap.h>
using namespace nox::app;
USING_NAMESPACE_NOX;

void FakeHDMap::Initialize()
{
    static ControlPanel panel;
    panel.Add({KeyBoard::R, KeyBoard::r}, "Send Map", [&]() {
        _want_send_map = true;
    });
}

void FakeHDMap::Process(optional<std_msgs::String> &hdmap)
{
    if(_want_send_map)
    {
        hdmap.emplace();
        file::Read("/home/yarten/Documents/planner0_out.xml", hdmap.value().data);
        _want_send_map = false;
    }
}
