#ifndef WAYPOINT_EDITOR_HPP
#define WAYPOINT_EDITOR_HPP

#include "waypoint.hpp"

#include <ros/ros.h>
#include <rviz/panel.h>

namespace rviz_plugins {

class WaypointEditor: public rviz::Panel
{
    Q_OBJECT

    public:

        WaypointEditor();
        ~WaypointEditor();

        void onInitialize() override;

    private:

        void load_waypoints();
        void publish_markers();

        Waypoints waypoints_;

        ros::NodeHandle nh_;
        ros::Publisher pub_;
};

}

#endif
