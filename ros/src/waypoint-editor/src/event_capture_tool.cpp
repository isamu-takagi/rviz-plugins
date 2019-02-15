#include "event_capture_tool.hpp"

#include <OGRE/OgreEntity.h>
#include <rviz/geometry.h>
#include <geometry_msgs/Point.h>

namespace rviz_plugins {

EventCapture::EventCapture()
{
    shortcut_key_ = 'c';
    property_.reset(new rviz::FloatProperty("Ground Height"));
    getPropertyContainer()->addChild(property_.get());
}

EventCapture::~EventCapture()
{
}

void EventCapture::onInitialize()
{
    move_tool_.initialize(context_);
    pub_ = nh_.advertise<geometry_msgs::Point>("/waypoint_editor/event", 1);
}

void EventCapture::activate()
{
    printf("activate\n");
}

void EventCapture::deactivate()
{
    printf("deactivate\n");
}

int EventCapture::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    if(event.rightDown())
    {
        Ogre::Plane ground(Ogre::Vector3::UNIT_Z, property_->getFloat());
        Ogre::Vector3 intersection;
        rviz::getPointOnPlaneFromWindowXY(event.viewport, ground, event.x, event.y, intersection);

        geometry_msgs::Point point;
        point.x = intersection.x;
        point.y = intersection.y;
        point.z = intersection.z;
        pub_.publish(point);

        printf("Mouse:(%d,%d) Ground:(%f,%f,%f)\n", event.x, event.y, intersection.x, intersection.y, intersection.z);
    }
    else
    {
        move_tool_.processMouseEvent(event);
        // setCursor( move_tool_.getCursor() );
    }
    return Render;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::EventCapture, rviz::Tool)
