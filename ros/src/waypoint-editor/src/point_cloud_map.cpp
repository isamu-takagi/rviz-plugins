#include "point_cloud_map.hpp"

#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rviz_plugins {

PointCloudMap::PointCloudMap()
{
    transform_.transform.translation.x = 0.0;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;
    transform_.transform.rotation.x = 0.0;
    transform_.transform.rotation.y = 0.0;
    transform_.transform.rotation.z = 0.0;
    transform_.transform.rotation.w = 1.0;
}

bool PointCloudMap::updateTransform()
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    {
        transform_ = tf_buffer.lookupTransform("map", "world", ros::Time::now(), ros::Duration(1.0));
    }
    catch (tf2::TransformException& exception)
    {
        ROS_WARN("failed to lookup transform: %s", exception.what());
    }
}

Point PointCloudMap::getGroundPoint(const Point& raypos, const Point& rayvec)
{
    Point gndpos = doTransform(raypos);
    gndpos.z = 0;
    return gndpos;
}

Point PointCloudMap::doTransform(const Point& point)
{
    geometry_msgs::Point in, out;
    in.x = point.x;
    in.y = point.y;
    in.z = point.z;
    tf2::doTransform(in, out, transform_);
    return Point{out.x, out.y, out.z};
}

}
