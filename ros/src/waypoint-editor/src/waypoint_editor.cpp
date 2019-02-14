#include "waypoint_editor.hpp"

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
}

WaypointEditor::~WaypointEditor()
{

}

void WaypointEditor::onInitialize()
{
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_editor/markers", 1);
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
    pub_.publish(msg);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::WaypointEditor, rviz::Panel)
