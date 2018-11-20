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

#include ".SimulatorModule.h"

namespace nox::app
{
    class Simulator
        : public SimulatorModule
    {
        /// Override your process functions ...

    protected:
        void Initialize() override;

        void
        Process(optional<geometry_msgs::PoseWithCovarianceStamped> obstacle, optional<nav_msgs::Odometry> vehicle_state,
                optional<nox_msgs::ObstacleArray> &obstacles,
                optional<traffic_light::msg_traffic_light_list> &traffic_lights) override;

        bool ProcessOnobstacle(geometry_msgs::PoseWithCovarianceStamped obstacle,
                               optional<nox_msgs::ObstacleArray> &obstacles,
                               optional<traffic_light::msg_traffic_light_list> &traffic_lights) override;

        bool ProcessOnvehicle_state(nav_msgs::Odometry vehicle_state, optional<nox_msgs::ObstacleArray> &obstacles,
                                    optional<traffic_light::msg_traffic_light_list> &traffic_lights) override;


    private:
        void Select(int index);

        void Add();

        void Add(const type::Obstacle &obstacle);

        void Delete();

        void Clear();

        void Move(double ds);

        void Turn(double da);

        void Shift(double dv);

        void StretchX(double dlx);

        void StretchY(double dly);

        void SwitchLight();

    private:
        void PrintHelp();

        void SetDefault(type::Obstacle &obstacle);

        void RefreshObstacle(type::Obstacle &obstacle);

    private:
        int _current_index = -1;
        std::vector<type::Obstacle> _obstacles;
        Pose _reference_pose;
        bool _traffic_light_green = true;
    };
}
