//
// Created by yarten on 18-11-20.
//


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Common/Common.h>

namespace nox::app
{
    void SendTF(const std::string &from, const std::string &to, const type::Pose &pose)
    {
        static tf::TransformBroadcaster br;

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pose.x, pose.y, pose.z) );
        transform.setRotation( tf::Quaternion(pose.r.x, pose.r.y, pose.r.z, pose.r.w) );

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), from, to));
    }
}
