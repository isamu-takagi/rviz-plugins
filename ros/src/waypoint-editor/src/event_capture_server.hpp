#ifndef EVENT_CAPTURE_SERVER_HPP
#define EVENT_CAPTURE_SERVER_HPP

#include <ros/ros.h>
#include <rviz/viewport_mouse_event.h>

namespace rviz_plugins {

class EventCaptureServer
{
    public:

        EventCaptureServer();
        ~EventCaptureServer();

        void initialize();
        void send(const rviz::ViewportMouseEvent& event);

    private:

        ros::NodeHandle nh_;
        ros::Publisher  pub_;
        ros::Subscriber sub_;
};

}

#endif
