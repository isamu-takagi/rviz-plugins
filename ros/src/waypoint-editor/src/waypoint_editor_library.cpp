#include "waypoint_editor_library.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace rviz_plugins {

WaypointEditorLibrary::WaypointEditorLibrary()
{
    select_radius_ = 0.5;
}

const Waypoints& WaypointEditorLibrary::getWaypoints() const
{
    return waypoints_;
}

void WaypointEditorLibrary::add(const Point& point)
{
    if(waypoints_.empty())
    {
        Waypoint waypoint;
        waypoint.pos = point;
        waypoints_.push_back(waypoint);
    }
    else
    {
        Point origin, vector;
        origin = waypoints_.back().pos;
        vector.x = point.x - origin.x;
        vector.y = point.y - origin.y;
        vector.z = point.z - origin.z;

        int loop = static_cast<int>(std::hypot(vector.x, vector.y)) + 1;
        for(int i = 1; i <= loop; ++i)
        {
            Waypoint waypoint;
            waypoint.pos.x = origin.x + (vector.x * i / loop);
            waypoint.pos.y = origin.y + (vector.y * i / loop);
            waypoint.pos.z = origin.z + (vector.z * i / loop);
            waypoints_.push_back(waypoint);
        }
    }
}

void WaypointEditorLibrary::select(const Point& point)
{
    double min_distance = 1e+10;
    selected_ = nullptr;

    for(auto& waypoint : waypoints_)
    {
        double distance = geometry_distance(point, waypoint.pos);
        if((distance < select_radius_) && (distance < min_distance))
        {
            min_distance = distance;
            selected_ = &waypoint;
        }
    }
}

void WaypointEditorLibrary::move(const Point& point)
{
    if(selected_)
    {
        selected_->pos = point;
    }
}

void WaypointEditorLibrary::release()
{
    selected_ = nullptr;
}

void WaypointEditorLibrary::load(const std::string& filepath)
{
    std::ifstream ifs(filepath.c_str());
    if(!ifs) { return; }

    std::string line, cell;
    getline(ifs, line); // Skip header line

    waypoints_.clear();
    while(getline(ifs, line), ifs)
    {
        std::istringstream iss(line);
        Waypoint w;
        getline(iss, cell, ',');  w.pos.x = std::stod(cell);
        getline(iss, cell, ',');  w.pos.y = std::stod(cell);
        getline(iss, cell, ',');  w.pos.z = std::stod(cell);
        getline(iss, cell, ',');  w.yaw = std::stod(cell);
        getline(iss, cell, ',');  w.vel = std::stod(cell);
        getline(iss, cell, ',');  w.change = std::stoi(cell);
        getline(iss, cell, ',');  w.event  = std::stoi(cell);
        getline(iss, cell, ',');  w.steer  = std::stoi(cell);
        getline(iss, cell, ',');  w.stop   = std::stoi(cell);
        waypoints_.push_back(w);
    }
}

void WaypointEditorLibrary::save(const std::string& filepath)
{
    std::ofstream ofs(filepath.c_str());
    if(!ofs) { return; }

    ofs << "x,y,z,yaw,velocity,change_flag,event_flag,steering_flag,stop_flag" << std::endl;
    ofs << std::fixed << std::setprecision(4);
    for(const auto& waypoint : waypoints_)
    {
        ofs << waypoint.pos.x  << ',';
        ofs << waypoint.pos.y  << ',';
        ofs << waypoint.pos.z  << ',';
        ofs << waypoint.yaw    << ',';
        ofs << waypoint.vel    << ',';
        ofs << waypoint.change << ',';
        ofs << waypoint.event  << ',';
        ofs << waypoint.steer  << ',';
        ofs << waypoint.stop   << std::endl;
    }
}

}
