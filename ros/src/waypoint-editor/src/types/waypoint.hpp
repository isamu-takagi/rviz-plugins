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

using Waypoints = std::vector<Waypoint>;

#endif
