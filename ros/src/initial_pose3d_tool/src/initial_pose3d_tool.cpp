#include "initial_pose3d_tool.hpp"


#include <rviz/display_context.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
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
    map_topic_property_ = new rviz::RosTopicProperty("Map Topic", "points_map", "Topic name of point cloud map", "type", auto_height_property_, SLOT(updateGround()), this);
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
    const auto point_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/points_map", ros::Duration(5.0));
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
    }
    else
    {
        ROS_WARN("failed to subscribe message: /points_map");
    }
}

void InitialPose3dTool::onPoseSet(double x, double y, double yaw)
{
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    geometry_msgs::TransformStamped transform;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    {
        transform = tf_buffer.lookupTransform(ground_frame_, fixed_frame, ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException& exception)
    {
        ROS_WARN("failed to lookup transform: %s", exception.what());
    }

    double z = height_property_->getFloat();
    double roll = roll_property_->getFloat();
    double pitch = pitch_property_->getFloat();

    geometry_msgs::Point fixed_position;
    geometry_msgs::Point ground_position;
    fixed_position.x = x;
    fixed_position.y = y;
    fixed_position.z = 0.0;

    tf2::doTransform(fixed_position, ground_position, transform);
    ground_position.z = 0.0;
    tf2::doTransform(ground_position, fixed_position, transform.inverse());

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = z;

    tf::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);

    ROS_INFO("Setting pose3d: %.3f %.3f %.3f %.3f %.3f %.3f [frame=%s]", x, y, z, roll, pitch, yaw, fixed_frame.c_str());
    pub_.publish(pose);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::InitialPose3dTool, rviz::Tool)
