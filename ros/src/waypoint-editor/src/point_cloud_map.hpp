#ifndef POINT_CLOUD_MAP_HPP
#define POINT_CLOUD_MAP_HPP

#include "types/geometry.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <boost/optional.hpp>
#include <map>
#include <string>

namespace rviz_plugins {

class PointCloudMap
{
    public:

        PointCloudMap();
        ~PointCloudMap() = default;

        std::string getTargetFrame();
        bool updateMap();
        boost::optional<Point> getGroundPoint(const Ray& ray);

    private:

        Point doTransform(const Point& point);

        float ground_height_global_;
        std::map<std::pair<int,int>,float> ground_height_;
        std::string target_frame_;
        geometry_msgs::TransformStamped transform_;
};

}

#endif
