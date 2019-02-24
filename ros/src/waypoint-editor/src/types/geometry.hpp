#ifndef TYPES_GEOMETRY_HPP
#define TYPES_GEOMETRY_HPP

struct Point
{
    double x, y, z;
};

struct MouseEvent
{
    Point raypos, rayvec, campos, camvec;
};

#endif
