#include <LoMap/tool/SceneOverlapBuilder.h>
#include <map>
USING_NAMESPACE_NOX;
using namespace nox::app;


SceneOverlapBuilder::SceneOverlapBuilder(Ptr<type::Scene> scene)
    : _scene(scene)
{
    assert(scene);
}

void SceneOverlapBuilder::Rebuild()
{
    // TODO: 在guide line上创造Overlap
}
