#include "event_capture_tool.hpp"

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
    if(event.right() || event.rightDown() || event.rightUp())
    {
        event_server_.send(event);
    }
    else
    {
        move_tool_.processMouseEvent(event);
    }
    return Render;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::EventCapture, rviz::Tool)
