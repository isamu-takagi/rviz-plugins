#include "initial_pose3d_tool.hpp"

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "rviz/display_context.h"

namespace rviz_plugins {

InitialPose3dTool::InitialPose3dTool()
{
    shortcut_key_ = '3';
    topic_property_ = new rviz::StringProperty("Topic", "initialpose3d", "Topic name of 3d pose estimate.", getPropertyContainer(), SLOT(updateTopic()), this);

    //auto_height_property_ = new rviz::BoolProperty("Auto Height", true, "Get height from point cloud map.", getPropertyContainer());
    auto_height_property_ = new rviz::BoolProperty("Auto Height", false, "Get height from point cloud map.", getPropertyContainer());
    auto_height_property_->setReadOnly(true);

    height_property_ = new rviz::FloatProperty("Height [m]", 0.0, "Height for pose estimate.", getPropertyContainer());
    roll_property_ = new rviz::FloatProperty("Roll [rad]", 0.0, "Roll for pose estimate.", getPropertyContainer());
    pitch_property_ = new rviz::FloatProperty("Pitch [rad]", 0.0, "Pitch for pose estimate.", getPropertyContainer());

}

void InitialPose3dTool::onInitialize()
{
    PoseTool::onInitialize();
    setName("3D Pose Estimate");
    updateTopic();
}

void InitialPose3dTool::updateTopic()
{
    pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_property_->getStdString(), 1);
}

void InitialPose3dTool::onPoseSet(double x, double y, double yaw)
{
    double z = height_property_->getFloat();
    double roll = roll_property_->getFloat();
    double pitch = pitch_property_->getFloat();

    std::string fixed_frame = context_->getFixedFrame().toStdString();
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = z;

    tf::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
    //pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    //pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    //pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
    ROS_INFO("Setting pose3d: %.3f %.3f %.3f %.3f %.3f %.3f [frame=%s]", x, y, z, roll, pitch, yaw, fixed_frame.c_str());
    pub_.publish(pose);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::InitialPose3dTool, rviz::Tool)
