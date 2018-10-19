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
#include "type/ReferenceLine.h"
#include "tool/TrajectoryStitcher.h"
#include "tool/TrafficDecider.h"
#include <Planner/impl/PlannerBase.h>
#include <memory>
#include <vector>

using std::vector;
using std::shared_ptr;

namespace nox::app
{
    class Planner
        : public PlannerModule
    {
        /// Override your process functions ...
    public:
        enum ErrorCode
        {
            Success,
            LackOfInformation,
            PlanningFail
        };

        using Result = container::Result<ErrorCode, ErrorCode::Success>;

    protected:
        void Initialize() override;

        void InitializeDeciders();

        void Process(nav_msgs::Odometry vehicle_state, optional<nox_msgs::Trajectory> &trajectory) override;

    public:
        /**
         * 进行规划流程处理
         * @param vehicle 车
         * @param scene  场景（包含车道线、障碍物、红绿灯灯物件）
         * @param result 传入的是空轨迹，或上一条轨迹，再使用之返回规划结果
         */
        Result Process(Ptr<type::Vehicle> vehicle, Ptr<type::Scene> scene, type::Trajectory & result);

    private:
        type::Vehicle _vehicle;
        type::Scene   _scene;

        vector< Ptr<DecisionMaker> > _deciders;
        Ptr<TrajectoryStitcher>      _trajectoryStitcher;
        Ptr<PlannerBase>             _algorithm;

        mailbox::Service<nox_msgs::GetScene> _scene_server;
    };
}
