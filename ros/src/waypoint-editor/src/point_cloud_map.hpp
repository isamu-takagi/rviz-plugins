#ifndef POINT_CLOUD_MAP_HPP
#define POINT_CLOUD_MAP_HPP

#include "types/geometry.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <map>

namespace rviz_plugins {

class PointCloudMap
{
    public:

        PointCloudMap();
        ~PointCloudMap() = default;

        bool updateMap();
        Point getGroundPoint(const Ray& ray);

    private:

        Point doTransform(const Point& point);

        geometry_msgs::TransformStamped transform_;
        float ground_height_global_;
        std::map<std::pair<int,int>,float> ground_height_;
};

}

#endif
