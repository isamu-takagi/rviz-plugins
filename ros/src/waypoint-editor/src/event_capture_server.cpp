#include "event_capture_server.hpp"

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <std_msgs/String.h>

namespace rviz_plugins
{

EventCaptureServer::EventCaptureServer()
{

}

EventCaptureServer::~EventCaptureServer()
{

}

void EventCaptureServer::initialize()
{
    pub_ = nh_.advertise<std_msgs::String>("/event_capture/event", 10);
    //sub_ = nh_.subscribe("/event_capture/command", 10, callback, this);
}

void EventCaptureServer::send(const rviz::ViewportMouseEvent &event)
{
    const auto viewport_x = event.x / static_cast<double>(event.viewport->getActualWidth());
    const auto viewport_y = event.y / static_cast<double>(event.viewport->getActualHeight());
    const auto camera = event.viewport->getCamera();
    const auto campos = camera->getPosition();
    const auto camvec = camera->getDirection();

    const auto ray = camera->getCameraToViewportRay(viewport_x, viewport_y);
    const auto raypos = ray.getOrigin();
    const auto rayvec = ray.getDirection();

    std_msgs::String msg;
    for(const auto& vec : {raypos, rayvec, campos, camvec})
    {
        msg.data += std::to_string(vec.x) + '\n';
        msg.data += std::to_string(vec.y) + '\n';
        msg.data += std::to_string(vec.z) + '\n';
    }
    pub_.publish(msg);
}

}
