#include "waypoint_editor_marker.hpp"
#include <visualization_msgs/MarkerArray.h>

namespace rviz_plugins {

WaypointEditorMarker::WaypointEditorMarker()
{
    latest_marker_size_ = 0;
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_editor/markers", 1);
}

void WaypointEditorMarker::publish(const Waypoints& waypoints, const std::string& frame)
{
    int marker_id = -1;
    visualization_msgs::MarkerArray msg;

    // Line
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "link";
        marker.id = ++marker_id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.1;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        for(const auto& waypoint : waypoints)
        {
            geometry_msgs::Point p;
            p.x = waypoint.pos.x;
            p.y = waypoint.pos.y;
            p.z = waypoint.pos.z;
            marker.points.push_back(p);
        }
        msg.markers.push_back(marker);
    }

    // Point
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "node";
        marker.id = ++marker_id;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        for(const auto& waypoint : waypoints)
        {
            geometry_msgs::Point p;
            p.x = waypoint.pos.x;
            p.y = waypoint.pos.y;
            p.z = waypoint.pos.z;
            marker.points.push_back(p);
        }

        msg.markers.push_back(marker);
    }

    // Text
    int csv_row = 1;
    for(const auto& waypoint : waypoints)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "text";
        marker.id = ++marker_id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.text = std::to_string(++csv_row);

        marker.pose.position.x = waypoint.pos.x;
        marker.pose.position.y = waypoint.pos.y;
        marker.pose.position.z = waypoint.pos.z + 1.0;
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

    int marker_size = latest_marker_size_;
    latest_marker_size_ = marker_id;

    // Remove
    while(marker_id < marker_size)
    {
        visualization_msgs::Marker marker;
        marker.ns = "text";
        marker.id = ++marker_id;
        marker.action = visualization_msgs::Marker::DELETE;


        msg.markers.push_back(marker);
        ROS_INFO("Remove %d", marker_id);
    }

    pub_.publish(msg);
}

}
