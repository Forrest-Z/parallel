/**
 * @brief 计算处理scene的各种动态物件重叠的信息
 */
#pragma once

#include <nox>
#include "SceneObjectCreator.h"

namespace nox::app
{
    class SceneOverlapBuilder
    {
    public:
        SceneOverlapBuilder(Ptr<type::Scene> scene);

        void Rebuild();

    private:
        Ptr<type::Scene> _scene;
        SceneObjectCreator<type::Overlap> _overlap_creator;
    };
}