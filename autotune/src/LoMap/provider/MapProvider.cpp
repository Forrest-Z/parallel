#include <LoMap/provider/MapProvider.h>
USING_NAMESPACE_NOX;
namespace nox::app
{

    void MapProvider::Update(const std::string &source)
    {
        _source.Update(source);
    }

    MD5<type::Map> MapProvider::Produce()
    {
        static std::hash<std::string> hasher;

        if(_source.IsFresh())
        {
            auto source = _source.Get();
            _map.reset(Map(), hasher(source));
            _map.data().From(source);

            Logger::D("MapProvider") << "Update Map. MD5: " << _map.md5();
        }

        return _map;
    }
}

