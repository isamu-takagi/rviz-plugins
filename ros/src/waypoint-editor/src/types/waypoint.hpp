#ifndef TYPES_WAYPOINT_HPP
#define TYPES_WAYPOINT_HPP

#include <vector>
#include "types/geometry.hpp"

struct Waypoint
{
    Point pos;
    double yaw;
    double vel;
    int change;
    int event;
    int steer;
    int stop;
};

struct EditPoint
{
    enum EditType {EDGE, CTRL, AUTO};
    EditType type;
};

using Waypoints = std::vector<Waypoint>;

#endif
