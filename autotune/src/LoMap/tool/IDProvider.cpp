#include <LoMap/tool/IDProvider.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


IDProvider::IDProvider(const scene::ID & init)
{
    if(init == scene::UNDEFINED_ID)
        Reset();
    else
        _current = init;
}

void IDProvider::Reset()
{
    _current = 0;
}

scene::ID IDProvider::Next()
{
    return _current++;
}
