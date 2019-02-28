#include "event_capture_server.hpp"

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <std_msgs/String.h>

namespace rviz_plugins
{

EventCaptureServer::EventCaptureServer()
{
    pub_ = nh_.advertise<std_msgs::String>("/event_capture/mouse", 10);
    //sub_ = nh_.subscribe("/event_capture/command", 10, callback, this);
}

EventCaptureServer::~EventCaptureServer()
{

}

void EventCaptureServer::send(rviz::ViewportMouseEvent &event)
{
    const auto viewport_x = event.x / static_cast<double>(event.viewport->getActualWidth());
    const auto viewport_y = event.y / static_cast<double>(event.viewport->getActualHeight());
    const auto camera = event.viewport->getCamera();
    const auto campos = camera->getPosition();
    const auto camvec = camera->getDirection();
    const auto ray = camera->getCameraToViewportRay(viewport_x, viewport_y);
    const auto raypos = ray.getOrigin();
    const auto rayvec = ray.getDirection();

    std::vector<bool> buttons;
    buttons.push_back(event.left());
    buttons.push_back(event.leftDown());
    buttons.push_back(event.leftUp());
    buttons.push_back(event.middle());
    buttons.push_back(event.middleDown());
    buttons.push_back(event.middleUp());
    buttons.push_back(event.right());
    buttons.push_back(event.rightDown());
    buttons.push_back(event.rightUp());
    buttons.push_back(event.alt());
    buttons.push_back(event.control());
    buttons.push_back(event.shift());

    std_msgs::String msg;
    for(const auto& vec : {raypos, rayvec, campos, camvec})
    {
        msg.data += std::to_string(vec.x) + ' ';
        msg.data += std::to_string(vec.y) + ' ';
        msg.data += std::to_string(vec.z) + '\n';
    }
    for(const auto& button : buttons)
    {
        msg.data += std::to_string(button) + ' ';
    }
    pub_.publish(msg);
}

}
