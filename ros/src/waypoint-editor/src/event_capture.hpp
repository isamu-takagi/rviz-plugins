#ifndef EVENT_CAPTURE_HPP
#define EVENT_CAPTURE_HPP

#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/default_plugin/tools/move_tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/properties/float_property.h>

#include <memory>

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
    private:

        rviz::MoveTool move_tool_;
        std::unique_ptr<rviz::FloatProperty> property_;

        ros::NodeHandle nh_;
        ros::Publisher  pub_;
};

}

#endif
