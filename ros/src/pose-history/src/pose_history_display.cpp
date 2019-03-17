#include "pose_history_display.hpp"

namespace rviz_plugins {

PoseHistory::PoseHistory()
{
    const char* topic_type = ros::message_traits::datatype<geometry_msgs::PoseStamped>();
    const char* topic_desc = "Name of topic to display";
    topic_property_.reset(new rviz::RosTopicProperty("Topic", "", topic_type, topic_desc, this, SLOT(updateTopic())));
}

PoseHistory::~PoseHistory()
{

}


void PoseHistory::updateTopic()
{
    unsubscribe();
    subscribe();
}

void PoseHistory::subscribe()
{
    auto topic_name = topic_property_->getTopicStd();
    if(1 < topic_name.length())
    {
        sub_ = nh_.subscribe(topic_name, 10, &PoseHistory::onMessage, this);
    }
}

void PoseHistory::unsubscribe()
{
    sub_.shutdown();
}

void PoseHistory::onMessage(const geometry_msgs::PoseStamped& message)
{
    ROS_INFO("On Message (%f, %f", message.pose.position.x, message.pose.position.y);
    history_.emplace_back(message);

    if(100 < history_.size())
    {
        history_.pop_front();
    }
}

void PoseHistory::onInitialize()
{
    lines_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
}

void PoseHistory::onEnable()
{
    ROS_INFO("On Enable");
    subscribe();
}

void PoseHistory::onDisable()
{
    ROS_INFO("On Disable");
}

void PoseHistory::update(float wall_dt, float ros_dt)
{
/*
    Ogre::Vector3 position, scale;
    Ogre::Quaternion orientation;
    Ogre::ColourValue color;
    transform(new_message, pos, orient, scale);
*/

    lines_->clear();
    lines_->setMaxPointsPerLine(history_.size());
    //lines_->setLineWidth(new_message->scale.x);
    //lines_->setPosition(position);
    //lines_->setOrientation(orientation);
    //lines_->setScale(scale);
    //lines_->setColor(r, g, b, a);

    for(const auto& message : history_)
    {
        Ogre::Vector3 point;
        point.x = message.pose.position.x;
        point.y = message.pose.position.y;
        point.z = message.pose.position.z;
        lines_->addPoint(point);
    }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseHistory, rviz::Display)
