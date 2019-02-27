#include "waypoint_editor_marker.hpp"
#include <visualization_msgs/MarkerArray.h>

namespace rviz_plugins {

WaypointEditorMarker::WaypointEditorMarker()
{
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_editor/markers", 1);
}

void WaypointEditorMarker::publish(const Waypoints& waypoints)
{
    const std::string frame = "/map";
    visualization_msgs::MarkerArray msg;

    // Line
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "link";
        marker.id = 0;
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
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
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
        marker.id = 1;
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
    if(false)
    {
        int count = 0;
        for(const auto& waypoint : waypoints)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame;
            marker.header.stamp = ros::Time::now();
            marker.ns = "text";
            marker.id = ++count;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            marker.text = std::to_string(count);

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
    }

    pub_.publish(msg);
}

}
