#include <Repeater/Repeater.h>
using namespace nox::app;
USING_NAMESPACE_NOX;


bool Repeater::ProcessOnjsk_obstacles(autoware_msgs::DetectedObjectArray jsk_obstacles,
                                      optional<nox_msgs::ObstacleArray> &obstacles)
{
    obstacles.emplace();
    obstacles.value().header = jsk_obstacles.header;

    for(auto & i : jsk_obstacles.objects)
    {
        nox_msgs::Obstacle obstacle;
        obstacle.header = i.header;
        obstacle.pose = i.pose;
        obstacle.length = i.dimensions;
        obstacle.vertexes = i.convex_hull.polygon;
        obstacle.speed = i.velocity.linear.x;
        obstacle.id = i.id;

        Pose pose;
        pose.From(obstacle.pose);

        if(real::IsZero(obstacle.speed))
        {
            nox_msgs::TrajectoryPoint point;
            pose.To(point.info.pose);
            point.v = 0;
            point.t = 0;
            obstacle.prediction.points.push_back(point);
            obstacle.speed = 0;
        }
        else
        {
            for(double t : range(0, 1, 8))
            {
                nox_msgs::TrajectoryPoint point;
                pose.Move(t * obstacle.speed).To(point.info.pose);
                point.t = t;
                point.v = obstacle.speed;
                obstacle.prediction.points.push_back(point);
            }
        }

        obstacles.value().obstacles.push_back(obstacle);
    }

    return true;
}
