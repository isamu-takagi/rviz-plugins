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

#endif
