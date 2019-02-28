#ifndef TYPES_CAPTURE_HPP
#define TYPES_CAPTURE_HPP

#include "types/geometry.hpp"

struct MouseEvent
{
    Ray select;
    Ray camera;
    bool left;
    bool left_down;
    bool left_up;
    bool middle;
    bool middle_down;
    bool middle_up;
    bool right;
    bool right_down;
    bool right_up;
    bool ctrl;
    bool shift;
    bool alt;
};

#endif
