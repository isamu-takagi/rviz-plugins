#include "event_capture_client.hpp"

#include <sstream>

namespace rviz_plugins {

EventCaptureClient::EventCaptureClient(): spinner_(1)
{
    //pub_ = nh_.advertise<std_msgs::String>("/event_capture/command", 10);
    sub_ = nh_.subscribe("/event_capture/mouse", 10, &EventCaptureClient::callbackMouseEvent, this);
    spinner_.start();
}

EventCaptureClient::~EventCaptureClient()
{
    spinner_.stop();
}

void EventCaptureClient::callbackMouseEvent(const std_msgs::String& msg)
{
    if(callback_mouse_)
    {
        std::istringstream iss(msg.data);
        MouseEvent event;
        iss >> event.select.origin.x    >> event.select.origin.y    >> event.select.origin.z;
        iss >> event.select.direction.x >> event.select.direction.y >> event.select.direction.z;
        iss >> event.camera.origin.x    >> event.camera.origin.y    >> event.camera.origin.z;
        iss >> event.camera.direction.x >> event.camera.direction.y >> event.camera.direction.z;
        iss >> event.left   >> event.left_down   >> event.left_up;
        iss >> event.middle >> event.middle_down >> event.middle_up;
        iss >> event.right  >> event.right_down  >> event.right_up;
        iss >> event.alt    >> event.ctrl        >> event.shift;
        callback_mouse_(event);
    }
}

}
