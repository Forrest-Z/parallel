#include <Tracking/component/PCPID.h>
#include "../../../../.param/template/Parameter.h"

USING_NAMESPACE_NOX;

namespace nox::app
{

    void PCPID::Initialize()
    {
        _param._pcpid = cache::ReadPCPIDParameter();
        _param._vehicle = cache::ReadVehicleParameter();
    }

    double PCPID::Calculate(const type::Trajectory &path, const type::Vehicle &vehicle)
    {
        const auto matched_point = path.PointAtPosition(vehicle.pose.t);
        const double v = vehicle.v.x;
        const double aError = vehicle.pose.theta - matched_point.pose.theta;
        const double dError = vehicle.pose.t.DistanceTo(matched_point.pose.t) *
            math::PointOnLine(vehicle.pose.x, vehicle.pose.y, matched_point.pose.x, matched_point.pose.y, matched_point.pose.theta);

        const double ff = FeedForward(v, -matched_point.kappa);
        const double fb = FeedBack(aError, dError);
        const double in = Integral(_integral, dError * v * 0.05 * std::cos(aError));
        _integral = in;

        double Kff, Kfb, Kin;
        PickPID(v, Kff, Kfb, Kin);

        const double sff = Kff * ff;
        const double sfb = Kfb * fb;
        const double sin = Kin * in;
        const double result = sff + sfb + sin;

        Logger::I("PCPID").Print(R"(
--------------------------------------------------------------------
[Input ]| %10s | %10s | %10s | %10s |
        | %10.6lf | %10.6lf | %10.6lf | %10.6lf |
--------------------------------------------------------------------
[ Item ]| %10s | %10s | %10s |
 weight | %10.6lf | %10.6lf | %10.6lf |
original| %10.6lf | %10.6lf | %10.6lf |
 in deg | %10.6lf | %10.6lf | %10.6lf |
   sum  | %10.6lf |
--------------------------------------------------------------------
)", "v", "aError/deg", "dError", "kappa",
v, aError * 180.0 / M_PI, dError, -matched_point.kappa,
"FF", "FB", "IN",
Kff, Kfb, Kin,
ff, fb, in,
ff * 180.0 / M_PI, fb * 180.0 / M_PI, in * 180.0 / M_PI,
result
);

        return result;
    }

    double PCPID::FeedForward(double v, double curvature)
    {
        double L  = _param._vehicle.Physical.WheelBase;
        double g  = 9.8;
        double Cf = _param._vehicle.Physical.Cornering.Front;
        double Cr = _param._vehicle.Physical.Cornering.Rear;
        double la = _param._vehicle.Physical.La;
        double lb = _param._vehicle.Physical.Lb;
        double m  = _param._vehicle.Physical.Weight;

        double Wf = lb / L * m * g;
        double Wr = la / L * m * g;
        double Kug = Wf / Cf - Wr / Cr;
        double heading = (L + Kug * v * v / g) * curvature;

        return heading;
    }

    double PCPID::FeedBack(double aError, double dError)
    {
        double ela = dError + _param._pcpid.xla * std::sin(aError);
        double heading = _param._pcpid.Kp / _param._vehicle.Physical.Cornering.Front * ela;

        return heading;
    }

    double PCPID::Integral(double last_value, double e)
    {
        double result = last_value + e;
        return result;
    }

    void PCPID::PickPID(double v, double & Kff, double & Kfb, double & Kin)
    {
        auto & vv = _param._pcpid.Kv;
        auto it = math::Extremum(vv.begin(), vv.end(), [&](const double & lhs, const double & rhs)
        {
            return std::abs(lhs - v) < std::abs(rhs - v);
        });

        size_t index = std::distance(vv.begin(), it);
        index = std::min<size_t>(index, vv.size() - 1);

        auto & vff = _param._pcpid.Kff;
        auto & vfb = _param._pcpid.Kfb;
        auto & vin = _param._pcpid.Kin;

        Kff = vff.empty() ? 0 : vff[std::min(index, vff.size() - 1)];
        Kfb = vfb.empty() ? 0 : vfb[std::min(index, vfb.size() - 1)];
        Kin = vin.empty() ? 0 : vin[std::min(index, vin.size() - 1)];
    }


}
