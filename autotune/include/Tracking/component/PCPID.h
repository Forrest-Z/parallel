/**
 * @file PCPID.h
 * 横向控制的实现，自行车模型的几何控制器+预描点PID
 */

#pragma once

#include <Tracking/component/interface/LateralController.h>
#include <Tracking/TrackingConfig.h>

namespace nox::app
{
    class PCPID
        : public LateralController
    {
    public:
        double Calculate(const type::Trajectory &path, const type::Vehicle &vehicle) override;

        void Initialize() override;

    private:
        double FeedForward(double v, double curvature);

        double FeedBack(double aError, double dError);

        double Damping(double vx, double vy, double w, double curvature, double aError);

        double Integral(double last_value, double e);

        void PickPID(double v, double & Kff, double & Kfb, double & Kin, double & Kdp);

    private:
        struct
        {
            parameter::VehicleParameter _vehicle;
            parameter::PCPIDParameter   _pcpid;
        } _param;

        double _integral = 0;
    };
}