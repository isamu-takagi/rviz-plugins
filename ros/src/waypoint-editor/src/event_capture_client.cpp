#include "event_capture_client.hpp"

#include <sstream>

namespace rviz_plugins
{

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
        iss >> event.raypos.x >> event.raypos.y >> event.raypos.z;
        iss >> event.rayvec.x >> event.rayvec.y >> event.rayvec.z;
        iss >> event.campos.x >> event.campos.y >> event.campos.z;
        iss >> event.camvec.x >> event.camvec.y >> event.camvec.z;
        callback_mouse_(event);
    }
}

}
