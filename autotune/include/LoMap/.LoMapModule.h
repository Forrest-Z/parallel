/**
 * @brief 用户模块框架代码，由noxcreate生成
 * @details
 * 1. 维护通信，参数读取，算法加载；
 * 2. 提供空实现的通信处理函数，可由子类重载；
 */
#pragma once

#include "LoMapConfig.h"

using std::optional;

namespace nox::app
{
    class LoMapModule
        : public manage::Module
    {
    protected: /// 用户处理代码，默认空实现
        virtual void Initialize();

        virtual void Terminate();

        virtual void Process( optional<nav_msgs::Odometry> vehicle_state, optional<nox_msgs::ObstacleArray> obstacles, optional<nox_msgs::Road> old_map, optional<std_msgs::String> hdmap, optional<nox_msgs::ObstacleArray> virtual_obstacles, optional<traffic_light::msg_traffic_light_list> traffic_lights, optional<nox_msgs::Trajectory> simple_map  );

        
        virtual bool ProcessOnvehicle_state( nav_msgs::Odometry vehicle_state   );
        virtual bool ProcessOnobstacles( nox_msgs::ObstacleArray obstacles   );
        virtual bool ProcessOnold_map( nox_msgs::Road old_map   );
        virtual bool ProcessOnhdmap( std_msgs::String hdmap   );
        virtual bool ProcessOnvirtual_obstacles( nox_msgs::ObstacleArray virtual_obstacles   );
        virtual bool ProcessOntraffic_lights( traffic_light::msg_traffic_light_list traffic_lights   );
        virtual bool ProcessOnsimple_map( nox_msgs::Trajectory simple_map   );

        

    protected: /// 参数成员、插件成员、信道成员
        struct
        {
            
            parameter::DeviceParameter Device;
        } params;

        struct
        {
            
        } plugins;

        struct
        {
            
            mailbox::Topic<nav_msgs::Odometry> vehicle_state;
            mailbox::Topic<nox_msgs::ObstacleArray> obstacles;
            mailbox::Topic<nox_msgs::Road> old_map;
            mailbox::Topic<std_msgs::String> hdmap;
            mailbox::Topic<nox_msgs::ObstacleArray> virtual_obstacles;
            mailbox::Topic<traffic_light::msg_traffic_light_list> traffic_lights;
            mailbox::Topic<nox_msgs::Trajectory> simple_map;
        } mailboxes;

    protected: /// 框架生命周期管理代码
        void OnStart() final;

        void OnRun() final;

        void OnFinish() final;

    private: /// 框架的配置接口
        void InitMailbox();

        void InitParameter();

        void InitCallback();

        void InitPlugin();

        void TerminateMailbox();

        void TerminatePlugin();

        void ProcessOutput(  );

    private: /// 框架成员
        struct
        {
            
        } loaders;

        struct
        {
            
        } postoffices;
    };
}
