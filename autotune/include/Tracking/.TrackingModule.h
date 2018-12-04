/**
 * @brief 用户模块框架代码，由noxcreate生成
 * @details
 * 1. 维护通信，参数读取，算法加载；
 * 2. 提供空实现的通信处理函数，可由子类重载；
 */
#pragma once

#include "TrackingConfig.h"

using std::optional;

namespace nox::app
{
    class TrackingModule
        : public manage::Module
    {
    protected: /// 用户处理代码，默认空实现
        virtual void Initialize();

        virtual void Terminate();

        virtual void Process( optional<nox_msgs::Trajectory> trajectory, nav_msgs::Odometry vehicle_state, nox_msgs::Chassis chassis,  optional<nox_msgs::DrivingCommand> & driving );

        

        
        void Onvehicle_stateFail( optional<nox_msgs::DrivingCommand> & driving );
        void OnchassisFail( optional<nox_msgs::DrivingCommand> & driving );

    protected: /// 参数成员、插件成员、信道成员
        struct
        {
            
            parameter::VehicleParameter Vehicle;
            parameter::VTFParameter VTF;
            parameter::PCPIDParameter PCPID;
        } params;

        struct
        {
            
        } plugins;

        struct
        {
            
            mailbox::Topic<nox_msgs::Trajectory> trajectory;
            mailbox::Topic<nav_msgs::Odometry> vehicle_state;
            mailbox::Topic<nox_msgs::Chassis> chassis;
            mailbox::Topic<nox_msgs::DrivingCommand> driving;
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

        void ProcessOutput( optional<nox_msgs::DrivingCommand> & driving );

    private: /// 框架成员
        struct
        {
            
        } loaders;

        struct
        {
            
        } postoffices;
    };
}
