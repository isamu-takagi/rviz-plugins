#include "event_capture.hpp"

namespace rviz_plugins {

EventCapture::EventCapture()
{
    printf("constructor\n");
}

EventCapture::~EventCapture()
{
    printf("destructor\n");
}

void EventCapture::onInitialize()
{
    printf("initialize\n");
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
    printf("mouse %d %d\n", event.x, event.y);
    return Render;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::EventCapture, rviz::Tool)