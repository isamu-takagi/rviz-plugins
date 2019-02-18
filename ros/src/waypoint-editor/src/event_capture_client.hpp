#ifndef EVENT_CAPTURE_CLIENT_HPP
#define EVENT_CAPTURE_CLIENT_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace rviz_plugins {

class EventCaptureClient
{
    public:

        EventCaptureClient();
        ~EventCaptureClient();

        void initialize();

    private:

        void on_receive(const std_msgs::String& msg);

        ros::AsyncSpinner spinner_;
        ros::NodeHandle nh_;
        ros::Publisher  pub_;
        ros::Subscriber sub_;
};

}

#endif
