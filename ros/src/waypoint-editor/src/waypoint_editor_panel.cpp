#include "waypoint_editor_panel.hpp"

#include <QLabel>
#include <QButtonGroup>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>

namespace rviz_plugins {

WaypointEditor::WaypointEditor()
{
    edit_mode_ = MODE_NONE;

    auto load_button = new QPushButton("Load");
    auto save_button = new QPushButton("Save");
    connect(load_button, &QPushButton::clicked, this, &WaypointEditor::loadWaypoints);
    connect(save_button, &QPushButton::clicked, this, &WaypointEditor::saveWaypoints);

    //auto mode_button_undo = new QPushButton("Undo");
    //auto mode_button_redo = new QPushButton("Redo");

    auto mode_button_mov = new QPushButton("Move");
    auto mode_button_add = new QPushButton("Add");
    auto mode_button_del = new QPushButton("Delete");
    mode_button_mov->setCheckable(true);
    mode_button_add->setCheckable(true);
    mode_button_del->setCheckable(true);

    auto mode_button_group = new QButtonGroup(this);
    mode_button_group->addButton(mode_button_mov, MODE_MOV);
    mode_button_group->addButton(mode_button_add, MODE_ADD);
    mode_button_group->addButton(mode_button_del, MODE_DEL);
    connect(mode_button_group, static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked), this, &WaypointEditor::changeEditMode);

    auto layout = new QVBoxLayout();
    setLayout(layout);
    layout->addWidget(new QLabel("Edit Mode"));
    layout->addWidget(mode_button_mov);
    layout->addWidget(mode_button_add);
    layout->addWidget(mode_button_del);
    //layout->addWidget(new QLabel("Command"));
    //layout->addWidget(mode_button_undo);
    //layout->addWidget(mode_button_redo);
    layout->addWidget(new QLabel("File Access"));
    layout->addWidget(load_button);
    layout->addWidget(save_button);
    layout->addStretch();
}

void WaypointEditor::onInitialize()
{
    point_cloud_map_.updateMap();
    capture_client_.setMouseEvent(this, &WaypointEditor::processMouseEvent);
}

void WaypointEditor::changeEditMode(int mode)
{
    edit_mode_ = static_cast<EditMode>(mode);
    std::cout << mode << std::endl;
}

void WaypointEditor::loadWaypoints()
{
    QString filepath = QFileDialog::getOpenFileName(this);
    if(!filepath.isEmpty())
    {
        waypoint_editor_.load(filepath.toStdString());
        waypoint_viewer_.publish(waypoint_editor_.getWaypoints(), point_cloud_map_.getTargetFrame());
    }
}

void WaypointEditor::saveWaypoints()
{
    QString filepath = QFileDialog::getSaveFileName(this);
    if(!filepath.isEmpty())
    {
        waypoint_editor_.save(filepath.toStdString());
    }
}

void WaypointEditor::processMouseEvent(const MouseEvent& event)
{
    if(event.shift)
    {
        return onSelect(event);
    }

    switch(edit_mode_)
    {
        case MODE_MOV: return onMove(event);
        case MODE_ADD: return onAdd(event);
        case MODE_DEL: return onDelete(event);
    }
}

void WaypointEditor::onSelect(const MouseEvent& event)
{

}

void WaypointEditor::onMove(const MouseEvent& event)
{
    if(event.right_down)
    {
        boost::optional<Point> gndpos = point_cloud_map_.getGroundPoint(event.select);
        if(gndpos)
        {
            waypoint_editor_.select(gndpos.get());
        }
    }
    else if(event.right)
    {
        boost::optional<Point> gndpos = point_cloud_map_.getGroundPoint(event.select);
        if(gndpos)
        {
            waypoint_editor_.move(gndpos.get());
            waypoint_viewer_.publish(waypoint_editor_.getWaypoints(), point_cloud_map_.getTargetFrame());
        }
    }
    else if(event.right_up)
    {
        waypoint_editor_.release();
    }
    else
    {

    }
}

void WaypointEditor::onAdd(const MouseEvent& event)
{
    if(event.right_down)
    {
        boost::optional<Point> gndpos = point_cloud_map_.getGroundPoint(event.select);
        if(gndpos)
        {
            waypoint_editor_.add(gndpos.get());
            waypoint_viewer_.publish(waypoint_editor_.getWaypoints(), point_cloud_map_.getTargetFrame());
        }
    }
}

void WaypointEditor::onDelete(const MouseEvent& event)
{
    if(event.right_down)
    {
        boost::optional<Point> gndpos = point_cloud_map_.getGroundPoint(event.select);
        if(gndpos)
        {
            waypoint_editor_.remove(gndpos.get());
            waypoint_viewer_.publish(waypoint_editor_.getWaypoints(), point_cloud_map_.getTargetFrame());
        }
    }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::WaypointEditor, rviz::Panel)
