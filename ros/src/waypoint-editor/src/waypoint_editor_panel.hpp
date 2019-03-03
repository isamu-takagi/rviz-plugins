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
        ~WaypointEditor() = default;

        void onInitialize() override;

    public Q_SLOTS:

        void changeEditMode(int mode);

    private:

        void loadWaypoints();
        void saveWaypoints();
        void processMouseEvent(const MouseEvent& event);

        void onSelect(const MouseEvent& event);
        void onMove(const MouseEvent& event);
        void onAdd(const MouseEvent& event);
        void onDelete(const MouseEvent& event);

        enum EditMode { MODE_NONE=0, MODE_MOV=1, MODE_ADD=2, MODE_DEL=3 };
        EditMode edit_mode_;

        EventCaptureClient capture_client_;
        PointCloudMap point_cloud_map_;
        WaypointEditorLibrary waypoint_editor_;
        WaypointEditorMarker waypoint_viewer_;
};

}

#endif
