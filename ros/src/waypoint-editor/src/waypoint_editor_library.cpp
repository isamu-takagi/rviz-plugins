#include "waypoint_editor_library.hpp"

#include <cmath>

namespace rviz_plugins {

const Waypoints& WaypointEditorLibrary::get() const
{
    return waypoints_;
}

void WaypointEditorLibrary::add(const Point& point)
{
    if(waypoints_.empty())
    {
        Waypoint waypoint;
        waypoint.x = point.x;
        waypoint.y = point.y;
        waypoint.z = point.z;
        waypoints_.push_back(waypoint);
    }
    else
    {
        Waypoint origin, vector;
        origin = waypoints_.back();
        vector.x = point.x - origin.x;
        vector.y = point.y - origin.y;
        vector.z = point.z - origin.z;

        int loop = static_cast<int>(std::hypot(vector.x, vector.y)) + 1;
        for(int i = 1; i <= loop; ++i)
        {
            Waypoint waypoint;
            waypoint.x = origin.x + (vector.x * i / loop);
            waypoint.y = origin.y + (vector.y * i / loop);
            waypoint.z = origin.z + (vector.z * i / loop);
            waypoints_.push_back(waypoint);
        }
    }
}

/*
void WaypointEditor::load_waypoints()
{
    QString filepath = QFileDialog::getOpenFileName(this);
    if(filepath.isEmpty())
    {
        return;
    }

    std::string line, cell;
    std::ifstream ifs(filepath.toStdString().c_str());

    // Skip header line
    getline(ifs, line);
    std::cout << line << std::endl;

    // Load waypoints
    while(getline(ifs, line), ifs)
    {
        std::istringstream iss(line);
        Waypoint w;
        getline(iss, cell, ',');  w.x   = std::stod(cell);
        getline(iss, cell, ',');  w.y   = std::stod(cell);
        getline(iss, cell, ',');  w.z   = std::stod(cell);
        getline(iss, cell, ',');  w.yaw = std::stod(cell);
        getline(iss, cell, ',');  w.vel = std::stod(cell);
        getline(iss, cell, ',');  w.change = std::stoi(cell);
        getline(iss, cell, ',');  w.event  = std::stoi(cell);
        getline(iss, cell, ',');  w.steer  = std::stoi(cell);
        getline(iss, cell, ',');  w.stop   = std::stoi(cell);
        waypoints_.push_back(w);
    }

    publish_markers();
}
*/

}
