/**
 * @file GuideLineBuilder.h
 */

#pragma once

#include <LoMap/type/ControlLine.h>

namespace nox::app
{
    class GuideLineBuilder
    {
    public:
        static void BuildPathUsingCubic(const ControlLine & controlLine, GuideLine & guideLine, double density = 0.3);

        static void BuildPathUsingCC_Dubins(const ControlLine & controlLine, GuideLine & guideLine);

        /**
         * 从control line的segments采样锚点，使用spline生成path
         * @bug 有点问题，生成不正确
         */
        static void BuildPathUsingSpline(const ControlLine & controlLine, GuideLine & guideLine);

        /**
         * 使用Cubie先生成path，再使用spline平滑它
         */
        static void BuildPathUsingSpline2(const ControlLine & controlLine, GuideLine & guideLine);

        static void BuildPathUsingSpline3(const ControlLine & controlLine, GuideLine & guideLine);

        /**
         * 在引导线上，根据控制线上的停止点建立停止线（只取最前边的停止线）
         * 要求：guideLine已经计算好path
         */
        static void BuildStopLine(const ControlLine & controlLine, GuideLine & guideLine);

        /**
         * 为引导线，根据control line的信息，创建减速地带
         * 要求：guideLine已经计算好path
         */
        static void BuildYellowZone(const ControlLine & controlLine, GuideLine & guideLine);

        /**
         * 为引导线，根据control line的信息，创建边界信息（地图边界、车道边界）
         * 要求：guideLine已经计算好path
         */
        static void BuildBoundary(const ControlLine & controlLine, GuideLine & guideLine);
    };
}