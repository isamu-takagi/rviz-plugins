#ifndef WAYPOINT_EDITOR_HPP
#define WAYPOINT_EDITOR_HPP


#include "event_capture_client.hpp"
#include "waypoint.hpp"


#include <ros/ros.h>
#include <rviz/panel.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

namespace rviz_plugins {

class WaypointEditor: public rviz::Panel
{
    Q_OBJECT

    public:

        WaypointEditor();
        ~WaypointEditor();

        void onInitialize() override;

    private:

        void onEventCapture(const geometry_msgs::Point& msg);
        void load_waypoints();
        void publish_markers();

        EventCaptureClient event_client_;

        Waypoints waypoints_;
        geometry_msgs::TransformStamped transform_;

        ros::NodeHandle nh_;
        ros::Publisher  pub_;
        ros::Subscriber sub_;
};

}

#endif
