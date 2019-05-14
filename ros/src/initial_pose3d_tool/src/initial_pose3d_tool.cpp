#include "initial_pose3d_tool.hpp"

#include <ros/console.h>
#include <rviz/display_context.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rviz_plugins {

InitialPose3dTool::InitialPose3dTool()
{
    shortcut_key_ = '3';

    pose_topic_property_ = new rviz::StringProperty("Pose Topic", "initialpose3d", "Topic name of 3d pose estimate.", getPropertyContainer(), SLOT(updateTopic()), this);
    height_property_ = new rviz::FloatProperty("Height [m]", 0.0, "Height for pose estimate.", getPropertyContainer());
    roll_property_ = new rviz::FloatProperty("Roll [rad]", 0.0, "Roll for pose estimate.", getPropertyContainer());
    pitch_property_ = new rviz::FloatProperty("Pitch [rad]", 0.0, "Pitch for pose estimate.", getPropertyContainer());
    auto_height_property_ = new rviz::BoolProperty("Auto Height", false, "Get height from point cloud map.", getPropertyContainer(), SLOT(updateGround()), this);
    map_topic_property_ = new rviz::RosTopicProperty("Map Topic", "/points_map", "Topic name of point cloud map", "sensor_msgs/PointCloud2", getPropertyContainer(), SLOT(updateGround()), this);
}

void InitialPose3dTool::onInitialize()
{
    PoseTool::onInitialize();
    setName("3D Pose Estimate");
    updateTopic();
    updateGround();
}

void InitialPose3dTool::updateTopic()
{
    pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_property_->getStdString(), 1);
}

void InitialPose3dTool::updateGround()
{
    ground_height_available_ = false;
    if(auto_height_property_->getBool() == false)
    {
        return;
    }

    ROS_INFO_STREAM("Load point cloud map");
    const auto point_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(map_topic_property_->getTopicStd(), ros::Duration(2.0));

    if(point_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> point_map;
        pcl::fromROSMsg(*point_msg, point_map);

        ground_frame_ = point_msg->header.frame_id;
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

        ground_height_available_ = true;
        ROS_INFO_STREAM(" * Num Points : " << point_map.size());
        ROS_INFO_STREAM(" * Num Areas  : " << ground_height_.size());
    }
    else
    {
        ROS_WARN_STREAM("failed to subscribe message: " + map_topic_property_->getTopicStd());
    }
}

void InitialPose3dTool::onPoseSet(double x, double y, double yaw)
{
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    tf2::Vector3 point(x, y, height_property_->getFloat());

    if(ground_height_available_)
    {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        tf2::Transform transform;
        try
        {
            const auto stamped = tf_buffer.lookupTransform(ground_frame_, fixed_frame, ros::Time(0), ros::Duration(1.0));
            tf2::fromMsg(stamped.transform, transform);
        }
        catch (tf2::TransformException& exception)
        {
            ROS_WARN_STREAM("failed to lookup transform: " << exception.what());
        }

        point = transform * point;

        auto key = std::make_pair(floor(point.getX()), floor(point.getY()));
        if(ground_height_.count(key))
        {
            point.setZ(ground_height_[key]);
        }

        point = transform.inverse() * point;
    }

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = fixed_frame;
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = point.getX();
    msg.pose.pose.position.y = point.getY();
    msg.pose.pose.position.z = point.getZ();

    double roll = roll_property_->getFloat();
    double pitch = pitch_property_->getFloat();
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    msg.pose.pose.orientation = tf2::toMsg(quaternion);

    ROS_INFO("Setting pose3d: %.3f %.3f %.3f %.3f %.3f %.3f [frame=%s]", point.x(), point.y(), point.z(), roll, pitch, yaw, fixed_frame.c_str());
    pub_.publish(msg);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::InitialPose3dTool, rviz::Tool)
