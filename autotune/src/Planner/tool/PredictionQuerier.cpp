#include <Planner/tool/PredictionQuerier.h>

using namespace nox::app;


PredictionQuerier::PredictionQuerier(nox::Ptr<nox::type::Scene> scene, ReferenceLine &reference)
    : _scene(scene), _reference(reference)
{
    // Do Nothing Here ...
}

double PredictionQuerier::ProjectVelocityAlongReferenceLine(const nox::scene::ID &obstacle_id, double s, double t)
{
    auto obstacle = _scene->GetObstacleByID(obstacle_id);
    assert(obstacle); // should has the obstacle
    
    auto point = obstacle->PointAtTime(t);
    double vx = point.v * std::cos(point.theta);
    double vy = point.v * std::sin(point.theta);
    
    auto nearest_point_path = _reference.path->PointAtDistance(s);
    double theta = nearest_point_path.theta;
    
    return std::cos(theta) * vx + std::sin(theta) * vy;
}
