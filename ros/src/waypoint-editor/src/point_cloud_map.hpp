#ifndef POINT_CLOUD_MAP_HPP
#define POINT_CLOUD_MAP_HPP

#include "types/geometry.hpp"
#include <geometry_msgs/TransformStamped.h>

namespace rviz_plugins {

class PointCloudMap
{
    public:

        PointCloudMap();
        ~PointCloudMap() = default;

        bool updateTransform();
        Point getGroundPoint(const Ray& ray);

    private:

        Point doTransform(const Point& point);
        geometry_msgs::TransformStamped transform_;
};

}

#endif
