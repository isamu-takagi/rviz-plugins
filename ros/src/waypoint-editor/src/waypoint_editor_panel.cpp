#include "waypoint_editor_panel.hpp"

#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>

#include <iostream>
#include <fstream>
#include <sstream>

namespace rviz_plugins {

WaypointEditor::WaypointEditor()
{
    auto layout = new QVBoxLayout();
    setLayout(layout);

    auto load_button = new QPushButton("Load");
    layout->addWidget(load_button);
    //connect(load_button, &QPushButton::clicked, this, &WaypointEditor::load_waypoints);

    layout->addStretch();
}

WaypointEditor::~WaypointEditor()
{

}

void WaypointEditor::onInitialize()
{
    capture_client_.setMouseEvent(this, &WaypointEditor::processMouseEvent);
}

#include <iostream>
void WaypointEditor::processMouseEvent(const MouseEvent& event)
{
    std::cout << event.raypos.x << " " << event.raypos.y << " " << event.raypos.z  << std::endl;
    std::cout << event.rayvec.x << " " << event.rayvec.y << " " << event.rayvec.z  << std::endl;
    std::cout << event.campos.x << " " << event.campos.y << " " << event.campos.z  << std::endl;
    std::cout << event.camvec.x << " " << event.camvec.y << " " << event.camvec.z  << std::endl; 

    Point gndpos = point_cloud_map_.getGroundPoint(event.raypos, event.rayvec);

    std::cout << gndpos.x << " " << gndpos.y << " " << gndpos.z  << std::endl; 

    waypoint_editor_.add(gndpos);
    waypoint_viewer_.publish(waypoint_editor_.get());
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::WaypointEditor, rviz::Panel)
