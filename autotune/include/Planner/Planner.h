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

#include ".PlannerModule.h"
#include <Planner/type/ReferenceLine.h>
#include <Planner/tool/TrajectoryStitcher.h>
#include <Planner/tool/DecisionMaker.h>
#include <Planner/PlannerBase.h>
#include <Planner/impl/BrakingPlanner.h>
#include <memory>
#include <vector>

using nox::container::Result;
using std::vector;
using std::shared_ptr;

namespace nox::app
{
    class Planner
        : public PlannerModule
    {
        /// Override your process functions ...

    protected:
        void Initialize() override;

        void InitializeDeciders();

        void InitializeParameters();

        void Process(nav_msgs::Odometry vehicle_state, optional<nox_msgs::Trajectory> &trajectory) override;

    public:
        Result<bool> Process(type::Trajectory & last_trajectory, type::Trajectory & result);

        Result<bool> CouldExtend(const PlannerBase::Frame &frame);

        /**
         * 进行规划流程处理
         * @param vehicle 车
         * @param scene  场景（包含车道线、障碍物、红绿灯灯物件）
         * @param new_trajectory 传入的是空轨迹，或上一条轨迹，再使用之返回规划结果
         */
        Result<bool> Plan(PlannerBase::Frame & frame, type::Trajectory &new_trajectory, bool should_replan = true);


    private:
        type::Vehicle _vehicle;
        type::Scene   _scene;

        vector< Ptr<DecisionMaker> > _deciders;
        Ptr<TrajectoryStitcher>      _trajectoryStitcher;
        Ptr<PlannerBase>             _algorithm;
        BrakingPlanner               _braking;

        mailbox::Service<nox_msgs::GetScene> _scene_server;
        mailbox::Topic<geometry_msgs::Point> _trajectory_plotter;

        struct
        {
            struct
            {
                double _replan_distance = 2.0;
                double _replan_time = 1.0;
                double _extend_time = 6.0;
                double _speed_diff  = 2.0;
            } _threshold;

            struct
            {
                double _backward_time = 1.0;
                double _forward_time = 4.0;
            } _reserve;
        } _param;
    };
}
