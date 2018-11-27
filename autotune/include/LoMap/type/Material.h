/**
 * @file Material.h
 */

#pragma once

#include <optional>
#include <nox>

namespace nox::app
{
    template <class T>
    class Material
        : public system::MutexLock
    {
    public:
        void Update(const T & data)
        {
            Synchronized(this)
            {
                if(not _data or _data.value() != data)
                {
                    _data.emplace(data);
                    _is_fresh = true;
                }
            }
        }

        bool IsFresh() const
        {
            Synchronized(this)
            {
                return _is_fresh;
            }
        }

        bool IsInit() const
        {
            Synchronized(this)
            {
                return bool(_data);
            }
        }

        const T & Get() const
        {
            Synchronized(this)
            {
                _is_fresh = false;
                return _data.value();
            }
        }

    private:
        std::optional<T> _data;
        mutable bool _is_fresh = false;
    };
}