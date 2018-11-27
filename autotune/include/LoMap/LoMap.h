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

#include ".LoMapModule.h"
#include <LoMap/tool/SceneMaintainer.h>
#include <LoMap/tool/SceneGenerator.h>

namespace nox::app
{
    class LoMap
        : public LoMapModule
    {
        /// Override your process functions ...
    protected:
        void Initialize() override;

        bool ProcessOnvehicle_state(nav_msgs::Odometry vehicle_state) override;

        bool ProcessOnobstacles(nox_msgs::ObstacleArray obstacles) override;

        bool ProcessOnold_map(nox_msgs::Road old_map) override;

        bool ProcessOnPlannerRequest(
            const nox_msgs::GetScene::Request & request,
            const mailbox::Address & address,
            nox_msgs::GetScene::Response & response);

        bool ProcessOnhdmap(std_msgs::String hdmap) override;

        bool ProcessOnvirtual_obstacles(nox_msgs::ObstacleArray virtual_obstacles) override;

        bool ProcessOntraffic_lights(traffic_light::msg_traffic_light_list traffic_lights) override;

    private:
        mailbox::Service<nox_msgs::GetScene> _scene_server;
        Ptr<SceneMaintainer> _scene_maintainer;
        SceneGenerator _scene_generator;
    };
}
