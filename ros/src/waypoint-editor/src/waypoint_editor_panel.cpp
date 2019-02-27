#include "waypoint_editor_panel.hpp"

#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>

namespace rviz_plugins {

WaypointEditor::WaypointEditor()
{
    auto layout = new QVBoxLayout();
    setLayout(layout);

    auto load_button = new QPushButton("Load");
    layout->addWidget(load_button);
    connect(load_button, &QPushButton::clicked, this, &WaypointEditor::load_waypoints);

    layout->addStretch();
}

WaypointEditor::~WaypointEditor()
{

}

void WaypointEditor::onInitialize()
{
    point_cloud_map_.updateMap();
    capture_client_.setMouseEvent(this, &WaypointEditor::processMouseEvent);
}

void WaypointEditor::load_waypoints()
{
    QString filepath = QFileDialog::getOpenFileName(this);
    if(!filepath.isEmpty())
    {
        waypoint_editor_.load(filepath.toStdString());
        waypoint_viewer_.publish(waypoint_editor_.get());
    }
}

void WaypointEditor::processMouseEvent(const MouseEvent& event)
{
    if(event.shift)
    {
        if(event.right_down)
        {
            Point gndpos = point_cloud_map_.getGroundPoint(event.select);
            waypoint_editor_.add(gndpos);
            waypoint_viewer_.publish(waypoint_editor_.get());
        }
    }
    else
    {
        if(event.right_down)
        {
            Point gndpos = point_cloud_map_.getGroundPoint(event.select);
            waypoint_editor_.select(gndpos);
        }
        else if(event.right)
        {
            Point gndpos = point_cloud_map_.getGroundPoint(event.select);
            waypoint_editor_.move(gndpos);
            waypoint_viewer_.publish(waypoint_editor_.get());
        }
        else if(event.right_up)
        {
            waypoint_editor_.release();
        }
        else
        {

        }
    }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::WaypointEditor, rviz::Panel)
