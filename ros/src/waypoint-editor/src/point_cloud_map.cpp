#include "point_cloud_map.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

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
    ground_height_global_ = 0.0;
}

bool PointCloudMap::updateMap()
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    {
        transform_ = tf_buffer.lookupTransform("map", "world", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException& exception)
    {
        ROS_WARN("failed to lookup transform: %s", exception.what());
    }

    const auto point_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/points_map", ros::Duration(1.0));
    if(point_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> point_map;
        pcl::fromROSMsg(*point_msg, point_map);

        ground_height_.clear();
        for(const auto& point : point_map)
        {
            auto key = std::make_pair(floor(point.x), floor(point.y));
            if(ground_height_.count(key) == 0)
            {
                ground_height_[key] = point.z;
            }
            else
            {
                ground_height_[key] = std::min(ground_height_[key], point.z);
            }
        }
    }
    else
    {
        ROS_WARN("failed to subscribe message: /points_map");
    }
}

Point PointCloudMap::getGroundPoint(const Ray& ray)
{
    Point gndpos = doTransform(ray.origin);

    auto key = std::make_pair(floor(gndpos.x), floor(gndpos.y));
    if(ground_height_.count(key))
    {
        gndpos.z = ground_height_[key];
    }
    else
    {
        gndpos.z = ground_height_global_;
    }
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
