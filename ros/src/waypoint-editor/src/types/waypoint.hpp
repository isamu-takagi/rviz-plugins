#ifndef TYPES_WAYPOINT_HPP
#define TYPES_WAYPOINT_HPP

#include <vector>

struct Waypoint
{
    double x;
    double y;
    double z;
    double yaw;
    double vel;
    int change;
    int event;
    int steer;
    int stop;
};

using Waypoints = std::vector<Waypoint>;

#endif
