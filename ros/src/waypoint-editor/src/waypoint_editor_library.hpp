#ifndef WAYPOINT_EDITOR_LIBRARY_HPP
#define WAYPOINT_EDITOR_LIBRARY_HPP

#include "types/geometry.hpp"
#include "types/waypoint.hpp"

namespace rviz_plugins {

class WaypointEditorLibrary
{
    public:

        WaypointEditorLibrary() = default;
        ~WaypointEditorLibrary() = default;

        const Waypoints& get() const;
        void add(const Point& point);
        void select(const Point& point);
        void move(const Point& point);
        void release();

    private:

        Waypoints waypoints_;
        Waypoint* selected_;
};

}

#endif
