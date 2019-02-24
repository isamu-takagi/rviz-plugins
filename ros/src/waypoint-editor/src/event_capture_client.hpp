#ifndef EVENT_CAPTURE_CLIENT_HPP
#define EVENT_CAPTURE_CLIENT_HPP

#include "types/geometry.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace rviz_plugins {

class EventCaptureClient
{
    public:

        EventCaptureClient();
        ~EventCaptureClient();

        template<class T>
        void setMouseEvent(T* obj, void(T::*func)(const MouseEvent& event))
        {
            callback_mouse_ = [=](const MouseEvent& event){ (obj->*func)(event); };
        }

    private:

        void callbackMouseEvent(const std_msgs::String& msg);
        //void callbackKeyEvent(const std_msgs::String& msg);

        ros::AsyncSpinner spinner_;
        ros::NodeHandle nh_;
        ros::Publisher  pub_;
        ros::Subscriber sub_;

        std::function<void(const MouseEvent& event)> callback_mouse_;
};

}

#endif
