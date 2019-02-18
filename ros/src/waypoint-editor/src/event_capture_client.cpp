#include "event_capture_client.hpp"

#include <thread>
#include <sstream>

namespace rviz_plugins
{

EventCaptureClient::EventCaptureClient(): spinner_(1)
{
    //pub_ = nh_.advertise<std_msgs::String>("/event_capture/command", 10);
    sub_ = nh_.subscribe("/event_capture/event", 10, &EventCaptureClient::on_receive, this);
    spinner_.start();
}

EventCaptureClient::~EventCaptureClient()
{
    spinner_.stop();
}

void EventCaptureClient::initialize()
{

}

void EventCaptureClient::on_receive(const std_msgs::String& msg)
{
    std::istringstream iss(msg.data);
    double x, y, z;
    for(int i = 0; i < 4; ++i)
    {
        iss >> x >> y >> z;
        printf("%f %f %f\n", x, y, z);
    }
    fflush(stdout);
}

}
