#ifndef WAYPOINT_EDITOR_LIBRARY_HPP
#define WAYPOINT_EDITOR_LIBRARY_HPP

#include "types/geometry.hpp"
#include "types/waypoint.hpp"

#include <string>

namespace rviz_plugins {

class WaypointEditorLibrary
{
    public:

        WaypointEditorLibrary();
        ~WaypointEditorLibrary() = default;

        void add(const Point& point);
        void select(const Point& point);
        void move(const Point& point);
        void release();

        const Waypoints& get() const;
        void load(const std::string& filepath);

    private:

        Waypoints waypoints_;
        Waypoint* selected_;

        double select_radius_;
};

}

#endif
