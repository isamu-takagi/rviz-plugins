#ifndef TYPES_GEOMETRY_HPP
#define TYPES_GEOMETRY_HPP

struct Point
{
    double x;
    double y;
    double z;
};

struct Ray
{
    Point origin;
    Point direction;
};

#include <cmath>

inline Point operator-(const Point& a, const Point& b)
{
    return Point{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline double geometry_distance(const Point& a, const Point& b)
{
    Point d = a - b;
    return sqrt((d.x * d.x) + (d.y * d.y) + (d.z * d.z));
}

#endif
