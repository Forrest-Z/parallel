#include <LoMap/tool/GuideLineBuilder.h>
#include <LoMap/provider/StopLineProvider.h>
#include <iostream>
#include <nox>

namespace nox::app
{
    void GuideLineBuilder::BuildPathUsingSpline(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine)
    {
        //region 初始化第一个控制点（之后迭代更新第二个点）
        auto first_line = controlLine->segments[0]->GetFunction();
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

        for(auto & i : controlLine->segments)
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
        guideLine->path = smoother.Smooth(anchors);
        //endregion
    }

    void GuideLineBuilder::BuildPathUsingCubic(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine, double density)
    {
        //region 初始化第一个控制点（之后迭代更新第二个点）
        auto first_line = controlLine->segments[0]->GetFunction();
        auto [x, y] = first_line->Calculate(0, 0);
        auto [dx, dy] = first_line->Calculate(1, 0);
        math::Derivative<1> x0, x1{x, dx}, y0, y1{y, dy};
        //endregion

        for(auto & i : controlLine->segments)
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
                guideLine->path.Add(point);

                size_t index = guideLine->path.Size() - 1;
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
                auto end = guideLine->path.Size();

                if(head_normal_index >= end)
                {
                    for(auto k : head_index)
                    {
                        guideLine->path[k].kappa = 0;
                        guideLine->path[k].dkappa = 0;
                    }
                }
                else
                {
                    auto & head = guideLine->path[head_normal_index];
                    for(auto k : head_index)
                    {
                        guideLine->path[k].kappa = head.kappa;
                        guideLine->path[k].dkappa = 0;
                    }
                }

            }
            //endregion

            //region 处理结束几个点曲率异常问题
            if(not tail_index.empty())
            {
                auto tail_size = tail_index.size();
                auto tail_normal_index = tail_index[0] - tail_size;
                auto end = guideLine->path.Size();

                if(tail_normal_index >= end)
                {
                    for(auto k : tail_index)
                    {
                        guideLine->path[k].kappa = 0;
                        guideLine->path[k].dkappa = 0;
                    }
                }
                else
                {
                    auto & tail = guideLine->path[tail_normal_index];
                    for(auto k : tail_index)
                    {
                        guideLine->path[k].kappa = tail.kappa;
                        guideLine->path[k].dkappa = 0;
                    }
                }
            }
            //endregion
        }
    }

    void GuideLineBuilder::BuildPathUsingCC_Dubins(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine)
    {
        std::vector<tool::AnchorPoint> anchors;
        anchors.emplace_back(controlLine->segments[0]->Front());

        for(auto & i : controlLine->segments)
        {
            anchors.emplace_back(i->Back());
        }

        tool::Smoother smoother;
        tool::Smoother::Config config;
        config.type = tool::Smoother::CC_Dubins;
        smoother.SetConfig(config);

        guideLine->path = smoother.Smooth(anchors);
    }

    void GuideLineBuilder::BuildStopLine(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine)
    {
        for(auto & i : controlLine->stopPoints)
        {
            auto   nearest_index = guideLine->path.QueryNearestByPosition(i);
            auto & nearest_point = guideLine->path[nearest_index];
            guideLine->SetStopLine(StopLine(nearest_point.s));
            guideLine->AddStopLine(StopLineProvider::DeadEnd, StopLine(nearest_point.s));
        }
    }

    void GuideLineBuilder::BuildPathUsingSpline2(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine)
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

        guideLine->path = smoother.Smooth(guideLine->path);
    }

    void GuideLineBuilder::BuildPathUsingSpline3(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine)
    {
        tool::Smoother::Config config;
        config.density = 0.3;
        config.type = tool::Smoother::Spline;
        config.anchor.density = 3.0;

        tool::Smoother smoother;
        smoother.SetConfig(config);

        BuildPathUsingCubic(controlLine, guideLine, 1);

    }


}

