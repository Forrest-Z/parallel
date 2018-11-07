/**
 * @brief 用户模块代码，由noxcreate生成
 * @details
 * 可以通过override处理函数来实现：
 * 1. 初始化
 * 2. 结束收尾
 * 3. 主处理函数
 * 4. 信道回调处理函数
 * 5. 信道不到位的回调处理函数
 *
 * 另外，其父类框架代码里边，有一些成员变量可以访问：
 * 1. 参数对象
 * 2. 插件对象
 * 3. 通信对象
 */
#pragma once

#include ".TrackingModule.h"
#include <Tracking/tool/LongitudinalController.h>
#include <Tracking/tool/LateralController.h>

namespace nox::app
{
    class Tracking
        : public TrackingModule
    {
    protected:
        void Initialize() override;

        void
        Process(optional<nox_msgs::Trajectory> trajectory, nav_msgs::Odometry vehicle_state, nox_msgs::Chassis chassis,
                optional<nox_msgs::DrivingCommand> &driving) override;

    private:
        Ptr<LongitudinalController> _lon_controller;
        Ptr<LateralController>      _lat_controller;
        Trajectory                  _trajectory;
        Vehicle                     _vehicle;
        system::Timer               _timer;
    };
}
