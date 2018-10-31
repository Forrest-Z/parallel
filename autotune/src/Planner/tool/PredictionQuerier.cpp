#include <utility>

#include <Planner/tool/PredictionQuerier.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


PredictionQuerier::PredictionQuerier(Ptr<type::Scene> scene, Ptr<ReferenceLine> reference)
    : _scene(std::move(scene)), _reference(std::move(reference))
{
    // Do Nothing Here ...
}

double PredictionQuerier::ProjectVelocityAlongReferenceLine(const scene::ID &obstacle_id, double s, double t)
{
    auto obstacle = _scene->Obstacles[obstacle_id];
    assert(obstacle); // should has the obstacle
    
    auto point = obstacle->PointAtTime(t);
    double vx = point.v * std::cos(point.pose.theta);
    double vy = point.v * std::sin(point.pose.theta);
    
    auto nearest_point_path = _reference->path.PointAtDistance(s);
    double theta = nearest_point_path.pose.theta;
    
    return std::cos(theta) * vx + std::sin(theta) * vy;
}
