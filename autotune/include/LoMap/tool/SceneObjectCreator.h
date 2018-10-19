/**
 * @brief 用于快速创建带ID物件
 */
#pragma once

#include <nox>
#include "IDProvider.h"
using namespace nox::type;

namespace nox::app
{
    template <class SceneObject>
    class SceneObjectCreator
    {
    public:
        Ptr<SceneObject> Create(const scene::ID & id = scene::UNDEFINED_ID);

    private:
        IDProvider _id_provider;
    };

    template<class SceneObject>
    Ptr<SceneObject> SceneObjectCreator<SceneObject>::Create(const scene::ID &id)
    {
        auto result = New<SceneObject>();

        if(id == scene::UNDEFINED_ID)
            result->id = _id_provider.Next();
        else
            result->id = id;

        return result;
    }
}