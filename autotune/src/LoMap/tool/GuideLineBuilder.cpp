#include <LoMap/tool/GuideLineBuilder.h>
#include <LoMap/LoMapConfig.h>
#include <iostream>
#include <nox>


namespace nox::app
{
    void GuideLineBuilder::BuildPathUsingSpline(const ControlLine & controlLine, GuideLine & guideLine)
    {
        //region 初始化第一个控制点（之后迭代更新第二个点）
        auto first_line = controlLine.segments[0]->GetFunction();
        auto [x, y] = first_line->Calculate(0, 0);
        auto [dx, dy] = first_line->Calculate(1, 0);
        math::Derivative<1> x0, x1{x, dx}, y0, y1{y, dy};
        //endregion

        //region 初始化锚点相关变量
        std::vector<tool::AnchorPoint> anchors;
        PathPoint last_point;
        double segment = 0;
        double total_sum = 0;
        bool is_finished = false;
        //endregion

        for(auto & i : controlLine.segments)
        {
            //region 获得当前分段
            auto s = i->Length();
            auto f = i->GetFunction();
            auto [tx0, ty0] = f->Calculate(0, 0);
            //endregion

            //region 处理连接处
            if(abs(tx0 - x1[0]) < 0.1 and abs(ty0 - y1[0]) < 0.1) // 连续不一定可导
            {
                auto [tdx0, tdy0] = f->Calculate(1, 0);
                x0 = {tx0, tdx0};
                y0 = {ty0, tdy0};
            }
            else
            {
                x0 = x1;
                y0 = y1;
            }
            //endregion

            //region 取合理的两端建立曲线
            auto [x, y] = f->Calculate(0, s);
            auto [dx, dy] = f->Calculate(1, s);

            x1 = {x, dx};
            y1 = {y, dy};

            math::CubicCurve curve_x(x0, x1, 1);
            math::CubicCurve curve_y(y0, y1, 1);
            //endregion

            //region 按步长取锚点
            for(double ds : range(0, 0.3, s - 0.3))
            {
                PathPoint current_point(curve_x, curve_y, ds / s);
                tool::AnchorPoint anchor(current_point);
                anchor.lateralBound = 0.1;
                anchor.longitudinalBound = 2.0;

                if(anchors.empty())
                {
                    anchor.s = 0;
                    anchors.push_back(anchor);
                }
                else
                {
                    segment += current_point.pose.t.DistanceTo(last_point.pose.t);
                    if(segment > 0.3)
                    {
                        total_sum += segment;
                        anchor.s = total_sum;
                        anchors.push_back(anchor);
                        segment = 0;

                        if(total_sum > 100)
                        {
                            is_finished = true;
                            break;
                        }
                    }
                }

                last_point = current_point;
            }
            //endregion

            if(is_finished) break;
        }

        //region 配置平滑器，并进行平滑
        tool::Smoother smoother;
        tool::Smoother::Config config;
        config.density = 0.3;
        config.type = tool::Smoother::Spline;
        config.anchor.density = 3.0;
        config.anchor.lateralBound = 0.1;
        config.anchor.longitudianlBound = 2;

        smoother.SetConfig(config);

        if(anchors.size() >= 2)
        {
            anchors.front().enforced = true;
            anchors.back().enforced = true;
        }
        guideLine.path = smoother.Smooth(anchors);
        //endregion
    }

    void GuideLineBuilder::BuildPathUsingCubic(const ControlLine & controlLine, GuideLine & guideLine, double density)
    {
        //region 初始化第一个控制点（之后迭代更新第二个点）
        auto first_line = controlLine.segments[0]->GetFunction();
        auto [x, y] = first_line->Calculate(0, 0);
        auto [dx, dy] = first_line->Calculate(1, 0);
        math::Derivative<1> x0, x1{x, dx}, y0, y1{y, dy};
        //endregion

        for(auto & i : controlLine.segments)
        {
            //region 获得当前分段
            auto s = i->Length();
            auto f = i->GetFunction();
            auto [tx0, ty0] = f->Calculate(0, 0);
            //endregion

            //region 处理连接处
            if(abs(tx0 - x1[0]) < 0.1 and abs(ty0 - y1[0]) < 0.1) // 连续不一定可导
            {
                auto [tdx0, tdy0] = f->Calculate(1, 0);
                x0 = {tx0, tdx0};
                y0 = {ty0, tdy0};
            }
            else
            {
                x0 = x1;
                y0 = y1;
            }
            //endregion

            //region 取合理的两端建立曲线
            auto [x, y] = f->Calculate(0, s);
            auto [dx, dy] = f->Calculate(1, s);

            x1 = {x, dx};
            y1 = {y, dy};

            math::CubicCurve curve_x(x0, x1, 1);
            math::CubicCurve curve_y(y0, y1, 1);
            //endregion

            //region 按步长取点，同时记录收尾处index
            std::vector<size_t> head_index, tail_index; // 记录轨迹开始几个点的下标
            for(double ds : range(0.0, density, s - density))
            {
                PathPoint point(curve_x, curve_y, ds / s);
                guideLine.path.Add(point);

                size_t index = guideLine.path.Size() - 1;
                if(ds < 3.0)
                    head_index.push_back(index);
                if(ds > s - 3.0)
                    tail_index.push_back(index);
            }
            //endregion

            //region 处理起始几个点曲率异常问题
            if(not head_index.empty())
            {
                auto head_size = head_index.size();
                auto head_normal_index = head_index[0] + head_size;
                auto end = guideLine.path.Size();

                if(head_normal_index >= end)
                {
                    for(auto k : head_index)
                    {
                        guideLine.path[k].kappa = 0;
                        guideLine.path[k].dkappa = 0;
                    }
                }
                else
                {
                    auto & head = guideLine.path[head_normal_index];
                    for(auto k : head_index)
                    {
                        guideLine.path[k].kappa = head.kappa;
                        guideLine.path[k].dkappa = 0;
                    }
                }

            }
            //endregion

            //region 处理结束几个点曲率异常问题
            if(not tail_index.empty())
            {
                auto tail_size = tail_index.size();
                auto tail_normal_index = tail_index[0] - tail_size;
                auto end = guideLine.path.Size();

                if(tail_normal_index >= end)
                {
                    for(auto k : tail_index)
                    {
                        guideLine.path[k].kappa = 0;
                        guideLine.path[k].dkappa = 0;
                    }
                }
                else
                {
                    auto & tail = guideLine.path[tail_normal_index];
                    for(auto k : tail_index)
                    {
                        guideLine.path[k].kappa = tail.kappa;
                        guideLine.path[k].dkappa = 0;
                    }
                }
            }
            //endregion
        }
    }

    void GuideLineBuilder::BuildPathUsingCC_Dubins(const ControlLine & controlLine, GuideLine & guideLine)
    {
        std::vector<tool::AnchorPoint> anchors;
        anchors.emplace_back(controlLine.segments[0]->Front());

        for(auto & i : controlLine.segments)
        {
            anchors.emplace_back(i->Back());
        }

        tool::Smoother smoother;
        tool::Smoother::Config config;
        config.type = tool::Smoother::CC_Dubins;
        smoother.SetConfig(config);

        guideLine.path = smoother.Smooth(anchors);
    }

    void GuideLineBuilder::BuildStopLine(const ControlLine & controlLine, GuideLine & guideLine)
    {
        for(auto & i : controlLine.stopPoints)
        {
            auto   nearest_index = guideLine.path.QueryNearestByPosition(i);
            auto & nearest_point = guideLine.path[nearest_index];
            guideLine.stop.Add(key::DeadEnd, nearest_point.s);
        }
    }

    void GuideLineBuilder::BuildPathUsingSpline2(const ControlLine & controlLine, GuideLine & guideLine)
    {
        BuildPathUsingCubic(controlLine, guideLine);

        tool::Smoother smoother;
        tool::Smoother::Config config;
        config.density = 0.3;
        config.type = tool::Smoother::Spline;
        config.anchor.density = 3.0;
        config.anchor.lateralBound = 0.1;
        config.anchor.longitudianlBound = 2;

        smoother.SetConfig(config);

        guideLine.path = smoother.Smooth(guideLine.path);
    }

    void GuideLineBuilder::BuildYellowZone(const ControlLine & controlLine, GuideLine & guideLine)
    {
        // FIXME：此处仅简单地限制路口处的速度（需要根据地图速度来调整）

        if(controlLine.junction_begin_index != -1)
        {
            auto & lane = *controlLine.segments[controlLine.junction_begin_index];
            auto frenet_end = guideLine.path.FrenetAtPosition(lane.Back().pose.t);

            guideLine.speed.Add(key::Junction,
                std::max(0.0, frenet_end.s - 40),
                frenet_end.s,
                0, 15.0 / 3.6
            );
        }

        if(controlLine.junction_end_index != -1)
        {
            auto & lane_begin = *controlLine.segments[controlLine.junction_begin_index + 1];
            auto & lane_end   = *controlLine.segments[controlLine.junction_end_index];
            auto frenet_begin = guideLine.path.FrenetAtPosition(lane_begin.Front().pose.t);
            auto frenet_end   = guideLine.path.FrenetAtPosition(lane_end.Back().pose.t);

            guideLine.speed.Add(key::Junction,
                frenet_begin.s, frenet_end.s,
                0, 15.0 / 3.6
            );
        }
    }

    void GuideLineBuilder::BuildBoundary(const ControlLine &controlLine, GuideLine &guideLine)
    {
        auto AddBoundary = [&](auto ptr)
        {
            auto front = ptr->Front();
            auto back  = ptr->Back();
            auto begin = guideLine.path.FrenetAtPosition(front.pose.t);
            auto end   = guideLine.path.FrenetAtPosition(back.pose.t);
            double wb = begin.l;
            double we = end.l;

            if(*ptr & Lane::Leftmost)
            {
                wb += front.width * 0.5;
                we += back.width * 0.5;
            }
            else
            {
                wb += front.width * -0.5;
                we += back.width * -0.5;
            }

            Boundary boundary;
            boundary.passable = false;
            boundary.s.Lower = begin.s;
            boundary.s.Upper = end.s;
            boundary.func.x = boundary.s;
            boundary.func.type = Function::Polynomial;

            if(real::IsEqual(wb, we))
                boundary.func.coeff = math::LinearCurve(
                    math::Derivative<0>{wb},
                    math::Derivative<0>{we},
                    begin.s,
                    end.s
                ).Coefficient();
            else
                boundary.func.coeff = math::CubicCurve(
                    math::Derivative<1>{wb, 0},
                    math::Derivative<1>{we, 0},
                    begin.s,
                    end.s
                ).Coefficient();

            guideLine.boundary.Add(key::MapEdge, boundary);
        };

        for(auto & i : controlLine.sections)
        {
            if(i != nullptr)
            {
                AddBoundary(i->Leftmost());
                AddBoundary(i->Rightmost());
            }
        }
    }

    void GuideLineBuilder::BuildPathUsingSpline(const vector<tool::AnchorPoint> & anchors, GuideLine &guideLine)
    {
        tool::Smoother smoother;
        tool::Smoother::Config config;
        config.density = 0.3;
        config.type = tool::Smoother::Spline;

        smoother.SetConfig(config);
        guideLine.path = smoother.Smooth(anchors);
    }


}

