#include "pose_history_display.hpp"

namespace rviz_plugins {

PoseHistory::PoseHistory()
{
    const char* topic_type = ros::message_traits::datatype<geometry_msgs::PoseStamped>();
    const char* topic_desc = "Name of topic to display";
    property_topic_ = new rviz::RosTopicProperty("Topic", "", topic_type, topic_desc, this, SLOT(updateTopic()));
    property_line_view_ = new rviz::BoolProperty("Line", true, "", this);
    property_line_width_ = new rviz::FloatProperty("Width", 0.1, "", property_line_view_);
    property_line_color_ = new rviz::ColorProperty("Color", Qt::white, "", property_line_view_);

    property_line_width_->setMin(0.0);
}

PoseHistory::~PoseHistory()
{
    // Properties are deleted by Qt
}


void PoseHistory::updateTopic()
{
    unsubscribe();
    subscribe();
}

void PoseHistory::subscribe()
{
    auto topic_name = property_topic_->getTopicStd();
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
    history_.emplace_back(message);
}

void PoseHistory::onInitialize()
{
    lines_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
}

void PoseHistory::onEnable()
{
    subscribe();
}

void PoseHistory::onDisable()
{
    unsubscribe();
}

void PoseHistory::update(float wall_dt, float ros_dt)
{
    if(100 < history_.size())
    {
        history_.pop_front();
    }


    QColor color = property_line_color_->getColor();
/*
    Ogre::Vector3 position, scale;
    Ogre::Quaternion orientation;
    transform(new_message, pos, orient, scale);
*/

    lines_->clear();
    lines_->setMaxPointsPerLine(history_.size());
    lines_->setLineWidth(property_line_width_->getFloat());
    //lines_->setPosition(position);
    //lines_->setOrientation(orientation);
    //lines_->setScale(scale);
    lines_->setColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());

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
