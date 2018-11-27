/**
 * @file SupplyChain.h 供应链模式用于订阅供应的类
 */

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <nox>
#include <map>
#include <condition_variable>

namespace nox::app
{
    template <class T>
    class SupplyChain
    {
    public:
        /**
         * 以一个名字购买产品，会等待到该产品被生产出来
         * @param name 产品名字
         * @return 产品
         */
        static T Buy(const std::string & name);

        /**
         * 以一个名字购买产品若干，会等待该产品满足数量才结束
         * @param name 产品名字
         * @param quantity 数量
         * @return 产品数组
         */
        static std::vector<T> Buy(const std::string & name, size_t quantity);

        /**
         * 以一个名字，销售T类型产品，并可指定生命长度
         * @param name 产品名字
         * @param data 产品
         * @param ttl  生命长度，若为-1，指无限长，否则，被访问到0结束
         */
        static void Sell(const std::string & name, const T & data, int ttl = -1);

    private:
        struct Product
        {
            T data;
            int ttl;    // 生命值，被访问一次减一，至零而死，负为永恒
        };

        class Warehouse
        {
            class Room
            {
            public:
                void AddProduct(const T & data, int ttl);

                void AddProduct(const std::vector<T> & data, int ttl);

                void TakeProduct(std::map<size_t, Ptr<T>> & list, size_t quantity);

            private:
                void _AddProduct(const T & data, int ttl);

                Ptr<T> _TakeProduct(size_t id);

            private:
                MutexLockable(std::map<size_t, Ptr<Product>>) _products;
                size_t _id = 0;
            };

        public:
            void AddProduct(const std::string & name, const T & data, int ttl);

            void AddProduct(const std::string & name, const std::vector<T> & data, int ttl);

            void TakeProduct(const std::string & name, std::map<size_t, Ptr<T>> & list, size_t quantity);

        private:
            Ptr<Room> GetRoom(const std::string & name);

        private:
            MutexLockable(std::unordered_map<std::string, Ptr<Room>>) _rooms;
        };

        static Warehouse _warehouse;
        static std::condition_variable _cv;
        static std::mutex _mtx;
    };

    template <class T>
    typename SupplyChain<T>::Warehouse SupplyChain<T>::_warehouse;

    template <class T>
    std::condition_variable SupplyChain<T>::_cv;

    template <class T>
    std::mutex SupplyChain<T>::_mtx;

    template<class T>
    T SupplyChain<T>::Buy(const std::string &name)
    {
        auto v = SupplyChain<T>::Buy(name, 1);
        return v[0];
    }

    template<class T>
    vector<T> SupplyChain<T>::Buy(const std::string &name, size_t quantity)
    {
        std::map<size_t, Ptr<T>> products;

        {
            std::unique_lock<std::mutex> lock(_mtx);
            while (products.size() < quantity)
            {
                _warehouse.TakeProduct(name, products, quantity);
                if(products.size() == quantity)
                    break;
                _cv.wait(lock);
            }
        }

        std::vector<T> r;
        for(auto & i : products)
            r.push_back(*i.second);
        return r;
    }

    template<class T>
    void SupplyChain<T>::Sell(const std::string &name, const T &data, int ttl)
    {
        std::unique_lock<std::mutex> lock(_mtx);
        _warehouse.AddProduct(name, data, ttl);
        _cv.notify_all();
    }

    template<class T>
    void SupplyChain<T>::Warehouse::Room::AddProduct(const T &data, int ttl)
    {
        Locking(_products)
        {
            _AddProduct(data, ttl);
        }
    }

    template<class T>
    void SupplyChain<T>::Warehouse::Room::AddProduct(const vector<T> &data, int ttl)
    {
        Locking(_products)
        {
            for(auto & i : data)
            {
                _AddProduct(data, ttl);
            }
        }
    }

    template<class T>
    void SupplyChain<T>::Warehouse::Room::_AddProduct(const T &data, int ttl)
    {
        _products[_id] = New<Product>();
        _products[_id].data = data;
        _products[_id].ttl = ttl;
//        ++_id; // TODO: 多产品模式有BUG，无法保证更替旧的产品，暂时仅维护一个产品
    }

    template<class T>
    void SupplyChain<T>::Warehouse::Room::TakeProduct(std::map<size_t, Ptr<T>> & list, size_t quantity)
    {
        Locking(_products)
        {
            quantity -= list.size();
            for(auto & i : _products)
            {
                if(quantity == 0)
                    break;

                size_t id = i.first;
                if(list.find(id) == list.end())
                {
                    list[id] = _TakeProduct(id);
                    --quantity;
                }
            }
        }
    }

    template<class T>
    Ptr<T> SupplyChain<T>::Warehouse::Room::_TakeProduct(size_t id)
    {
        auto product = _products[id];
        assert(product);
        assert(product->ttl != 0);

        auto r = New<T>(product->data);

        if(product->ttl > 0 and --product->ttl == 0)
        {
            _products.erase(id);
        }

        return r;
    }

    template<class T>
    Ptr<typename SupplyChain<T>::Warehouse::Room> SupplyChain<T>::Warehouse::GetRoom(const std::string &name)
    {
        Locking(_rooms)
        {
            if(_rooms[name] == nullptr)
                _rooms[name] = New<Room>();
            return _rooms[name];
        }
    }

    template<class T>
    void SupplyChain<T>::Warehouse::AddProduct(const std::string &name, const T &data, int ttl)
    {
        auto room = GetRoom(name);
        room->AddProduct(data, ttl);
    }

    template<class T>
    void SupplyChain<T>::Warehouse::AddProduct(const std::string &name, const vector<T> &data, int ttl)
    {
        auto room = GetRoom(name);
        room->AddProduct(data, ttl);
    }

    template<class T>
    void SupplyChain<T>::Warehouse::TakeProduct(const std::string & name, map<size_t, Ptr<T>> &list, size_t quantity)
    {
        auto room = GetRoom(name);
        room->TakeProduct(list, quantity);
    }
}