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
        static void BuildPathUsingCubic(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine);

        static void BuildPathUsingCC_Dubins(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine);

        /**
         * 从control line的segments采样锚点，使用spline生成path
         * @bug 有点问题，生成不正确
         */
        static void BuildPathUsingSpline(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine);

        /**
         * 使用Cubie先生成path，再使用spline平滑它
         */
        static void BuildPathUsingSpline2(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine);

        /**
         * 在引导线上，根据控制线上的停止点建立停止线（只取最前边的停止线）
         * 要求：guideLine已经计算好path
         */
        static void BuildStopLine(Ptr<ControlLine> controlLine, Ptr<GuideLine> guideLine);
    };
}