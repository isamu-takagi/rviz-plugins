#ifndef EVENT_CAPTURE_HPP
#define EVENT_CAPTURE_HPP

//#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>

namespace rviz_plugins {

class EventCapture: public rviz::Tool
{
    Q_OBJECT

    public:

        EventCapture();
        ~EventCapture();

        void onInitialize() override;
        void activate() override;
        void deactivate() override;
        int processMouseEvent(rviz::ViewportMouseEvent& event) override;
};

}

#endif
