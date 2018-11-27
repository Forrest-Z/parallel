//
// Created by yarten on 18-11-26.
//

#pragma once

#include <nox>

namespace nox::app
{
    /**
     * @class MD5
     * @brief 使用一个md5值，来判断对象是否相等，该类维护的数据为指针，为的是快速浅拷贝
     */
    template <class T>
    class MD5
    {
    public:
        template <class ... Args>
        MD5(Args && ... args)
            : _data(New<T>(args...))
        {}

        MD5(const MD5 & other) = default;

        MD5(MD5 & other) = default;

        MD5 &operator=(const MD5 & other) = default;

        T & data()
        {
            return *_data;
        }

        const T & data() const
        {
            return *_data;
        }

        void reset(const T & data_, size_t md5_)
        {
            _data = New<T>(data_);
            _md5 = md5_;
        }

        size_t & md5()
        {
            return _md5;
        }

        size_t md5() const
        {
            return _md5;
        }

        bool operator==(const MD5<T> & other)
        {
            return _md5 == other._md5;
        }

        bool operator!=(const MD5<T> & other)
        {
            return not operator==(other);
        }

    private:
        size_t _md5 = 0;
        Ptr<T> _data;
    };
}