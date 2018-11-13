#include <Tracking/tool/VTF.h>
#include <Tracking/TrackingConfig.h>
#include "../../../../.param/template/Parameter.h"

USING_NAMESPACE_NOX;
using namespace nox::app;

VTF::VTF()
{
    filter::RC_1o_1d::Config da_config;
    da_config.ThD = Degree(10).Get<Radian>();
    da_config.ThC = 20;

    _da_filter.SetConfig(da_config);
}


double VTF::Calculate(const type::Trajectory &path, const type::Vehicle &vehicle)
{
    /// -----------------------------------------------------------------------------
    /// 数据初始化
    auto & vehicle_param = cache::ReadVehicleParameter();
    auto & vtf_param = cache::ReadVTFParameter();
    auto nearest_index = path.QueryNearestByPosition(vehicle.pose.t);
    auto nearest_point = path[nearest_index];
    auto nearest_frenet = path.FrenetAtPosition(vehicle.pose.t);

    double dError = nearest_frenet.l;
    double aError = vehicle.pose.theta - nearest_frenet.theta; // nearest_point.pose.theta;
    double v = vehicle.v.x;

    // aError = _da_filter(aError);

    Logger::I("Map").Print("Map(x, y, theta, kappa): %6.3lf m, %6.3lf m, %6.3lf deg, %10.6lf 1/m",
                           nearest_point.pose.x.get(),
                           nearest_point.pose.y.get(),
                           nearest_point.pose.theta.get(),
                           nearest_point.kappa);

    Logger::I("VTF").Print("Input(dError, aError): %6.3lf m, %6.3lf deg",
                           dError,
                           aError * 180.0 / M_PI);

    /// -----------------------------------------------------------------------------
    /// 开始VTF计算
    double eyt = dError;
    double delta_psi0 = aError;
    double vx = std::max(v, 3.5);
    double vx1 = v;
    
    if(v == 0) vx = 0;
    double Kappa = -nearest_point.kappa;
    double eps = real::Epsilon;
    double m = vehicle_param.Physical.Weight;
    double hc = vehicle_param.Physical.Height * 0.5;
    double c = vehicle_param.Physical.Width * 0.5;
    double a = vehicle_param.Physical.La;
    double b = vehicle_param.Physical.Lb;
    double L = a + b;
    double Ca1 = vehicle_param.Physical.Cornering.Front * 2;
    double Ca2 = vehicle_param.Physical.Cornering.Rear * 2;
    double lastSteering = Degree(vehicle.steering / vehicle_param.Steering.Ratio).Get<Radian>();

    if(vtf_param.Option.Kappa_filter && abs(Kappa) < vtf_param.kappa_min)
        Kappa = 0;

    double ls = std::min(vtf_param.lsmax, std::max(vtf_param.lsmin /* 5 */, vx1 * vtf_param.lsmulti));
    double eymax = (vx1 * 3.6 - 40.0) * ( vtf_param.eymax_1 - vtf_param.eymax_2 ) / ( 10.0 - 40.0 ) + vtf_param.eymax_2;
    eymax = std::max(eymax, vtf_param.eymax_min);

    if(vx1 <= vtf_param.eymax_speed / 3.6) eymax = vtf_param.eymax_low_speed;
    double kPR = 1 * atan(c / hc) / (2 * eymax);
    double thetamax = atan(c / hc);
    double K1 = a * m / (L * Ca2);
    double K2 = m / (L * L) * (a / Ca2 - b / Ca1);

    double Steer_comp = vtf_param.Option.Steer_comp; /// 道路侧倾控制时1，不控制时为0
    double R_road = vtf_param.Option.R_road; /// 转向补偿控制时1，不控制时为0

    double g = 9.8;
    double delta_psi1 = (b + K1 * vx1 * vx1) * Kappa * Steer_comp;
    double delta_psi = sin(delta_psi0 + delta_psi1);
    double Rref = 1.0 / (abs(Kappa) + eps); // 圆周运动的半径
    double thetax = - 2 * kPR * (eyt + ls * delta_psi) - R_road * atan(vx1 * vx1 * Kappa / g);

    double theta = std::min(abs(thetax), thetamax) * math::Sign(thetax);
    double Kbeta = (b + K1 * vx * vx) / ((1 + K2 * vx * vx) * L + eps);
    double Kw = vx / ((1 + K2 * vx * vx) * L + eps);
    double faiytheta = -(m * g * sin(theta)) / ((Ca1 + Ca2) * Kbeta + (Ca1 * a - Ca2 * b) * Kw / (vx + eps) - Ca1 + eps);
    double faiR = L * (1 + K2 * vx1 * vx1) * Kappa;
    double fai_target0 = faiytheta + faiR;
    double Rmin = (vx1 * vx1 * hc) / (g * c);
    double fai_max = L * (1 + K2 * vx1 * vx1) / (Rmin + eps);

    double fai_target = std::min(abs(fai_target0), fai_max) * math::Sign(fai_target0);

    return fai_target;
}


