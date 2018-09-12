#include <Planner/type/Frame.h>
#include <nox>
USING_NAMESPACE_NOX;
using namespace nox::app;

static size_t FrameIDCounter = 0;

Frame::Frame()
    : _ID(FrameIDCounter++)
{

}

size_t Frame::ID() const
{
    return _ID;
}

string Frame::Brief() const
{
    return FormatOut("[Frame: ID = %lu]", _ID);
}
