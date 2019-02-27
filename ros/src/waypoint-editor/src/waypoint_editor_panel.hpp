#ifndef WAYPOINT_EDITOR_PANEL_HPP
#define WAYPOINT_EDITOR_PANEL_HPP

#include "event_capture_client.hpp"
#include "point_cloud_map.hpp"
#include "waypoint_editor_library.hpp"
#include "waypoint_editor_marker.hpp"

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
        void processMouseEvent(const MouseEvent& event);

        EventCaptureClient capture_client_;
        PointCloudMap point_cloud_map_;
        WaypointEditorLibrary waypoint_editor_;
        WaypointEditorMarker waypoint_viewer_;
};

}

#endif
