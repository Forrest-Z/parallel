/**
 * @brief scene::ID分发者
 */
#pragma once

#include <nox>

namespace nox::app
{
    class IDProvider
    {
    public:
        IDProvider(const scene::ID & init = scene::UNDEFINED_ID);

        void Reset();

        scene::ID Next();

    private:
        scene::ID _current;
    };
}