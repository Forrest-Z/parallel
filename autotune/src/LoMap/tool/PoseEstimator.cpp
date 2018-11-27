#include <utility>
#include <LoMap/tool/PoseEstimator.h>
USING_NAMESPACE_NOX;
using namespace nox::app;

PoseEstimator::PoseEstimator()
{
    _history.SetBufferSize(1);

    Odometry init_odometry;
    init_odometry.timeStamp = Clock::now();
    _history.Fill(init_odometry);
}

void PoseEstimator::Update(const Odometry & stamped_odometry)
{
    _history.Push(stamped_odometry);
}

Pose PoseEstimator::Estimate(const type::Time &current_time)
{
    return _history.Lattest().pose;

    auto state = _history.Lattest();
    double t = current_time.Get(Time::Second) - state.timeStamp.Get(Time::Second);
    double v = state.v.x;
    double w = state.w.z;
    double theta = 0;

    Vector3 dP;
    if(Real::IsZero(w))
    {
        dP.x = v * t;
    }
    else
    {
        double R = v / w;
        theta = w * t;
        dP.x = -R * (1 - std::cos(theta));
        dP.y = R * std::sin(theta);
    }

    Position future_position = state.pose.r * dP + state.pose.t;
    Rotation future_rotation(state.pose.theta + theta);

    return Pose(future_position, future_rotation);
}
