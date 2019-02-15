#include "waypoint_editor.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>

#include <iostream>
#include <fstream>
#include <sstream>

namespace rviz_plugins {

WaypointEditor::WaypointEditor()
{
    auto layout = new QVBoxLayout();
    setLayout(layout);

    auto load_button = new QPushButton("Load");
    layout->addWidget(load_button);
    connect(load_button, &QPushButton::clicked, this, &WaypointEditor::load_waypoints);

    layout->addStretch();
}

WaypointEditor::~WaypointEditor()
{

}

void WaypointEditor::onInitialize()
{
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_editor/markers", 1);
    sub_ = nh_.subscribe("/waypoint_editor/event", 1, &WaypointEditor::onEventCapture, this);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    {
        transform_ = tf_buffer.lookupTransform("map", "world", ros::Time::now(), ros::Duration(1.0));
    }
    catch (tf2::TransformException& exception)
    {
        setEnabled(false);
        ROS_WARN("failed to lookup transform: %s", exception.what());
    }
}

void WaypointEditor::onEventCapture(const geometry_msgs::Point& msg)
{
    geometry_msgs::Point point;
    tf2::doTransform(msg, point, transform_);

    if(waypoints_.empty())
    {
        Waypoint waypoint;
        waypoint.x = point.x;
        waypoint.y = point.y;
        waypoint.z = point.z;
        waypoints_.push_back(waypoint);
    }
    else
    {
        Waypoint origin, vector;
        origin = waypoints_.back();
        vector.x = point.x - origin.x;
        vector.y = point.y - origin.y;
        vector.z = point.z - origin.z;

        int loop = static_cast<int>(hypot(vector.x, vector.y)) + 1;
        for(int i = 1; i <= loop; ++i)
        {
            Waypoint waypoint;
            waypoint.x = origin.x + (vector.x * i / loop);
            waypoint.y = origin.y + (vector.y * i / loop);
            waypoint.z = origin.z + (vector.z * i / loop);
            waypoints_.push_back(waypoint);
        }
    }
    publish_markers();
}

void WaypointEditor::load_waypoints()
{
    QString filepath = QFileDialog::getOpenFileName(this);
    if(filepath.isEmpty())
    {
        return;
    }

    std::string line, cell;
    std::ifstream ifs(filepath.toStdString().c_str());

    // Skip header line
    getline(ifs, line);
    std::cout << line << std::endl;

    // Load waypoints
    while(getline(ifs, line), ifs)
    {
        std::istringstream iss(line);
        Waypoint w;
        getline(iss, cell, ',');  w.x   = std::stod(cell);
        getline(iss, cell, ',');  w.y   = std::stod(cell);
        getline(iss, cell, ',');  w.z   = std::stod(cell);
        getline(iss, cell, ',');  w.yaw = std::stod(cell);
        getline(iss, cell, ',');  w.vel = std::stod(cell);
        getline(iss, cell, ',');  w.change = std::stoi(cell);
        getline(iss, cell, ',');  w.event  = std::stoi(cell);
        getline(iss, cell, ',');  w.steer  = std::stoi(cell);
        getline(iss, cell, ',');  w.stop   = std::stoi(cell);
        waypoints_.push_back(w);
    }

    publish_markers();
}

void WaypointEditor::publish_markers()
{
    visualization_msgs::MarkerArray msg;

    // Points
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "node";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        for(const auto& waypoint : waypoints_)
        {
            geometry_msgs::Point p;
            p.x = waypoint.x;
            p.y = waypoint.y;
            p.z = waypoint.z;
            marker.points.push_back(p);
        }

        msg.markers.push_back(marker);
    }

    // Text
    {
        int count = 0;
        for(const auto& waypoint : waypoints_)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "text";
            marker.id = ++count;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            marker.text = std::to_string(count);

            marker.pose.position.x = waypoint.x;
            marker.pose.position.y = waypoint.y;
            marker.pose.position.z = waypoint.z + 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.0;
            marker.scale.y = 0.0;
            marker.scale.z = 0.5;

            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            msg.markers.push_back(marker);
        }
    }

    pub_.publish(msg);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::WaypointEditor, rviz::Panel)
