#ifndef POSE_HISTORY_HPP
#define POSE_HISTORY_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>

#include <rviz/ogre_helpers/billboard_line.h>
#include <deque>
#include <memory>

namespace rviz_plugins {

class PoseHistory : public rviz::Display
{
    Q_OBJECT

    public:

        PoseHistory();
        virtual ~PoseHistory();

    protected:

        void onInitialize() override;
        void onEnable() override;
        void onDisable() override;
        void update(float wall_dt, float ros_dt) override;

    private Q_SLOTS:

        void updateTopic();
        void subscribe();
        void unsubscribe();
        void onMessage(const geometry_msgs::PoseStamped& message);

    private:

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        std::deque<geometry_msgs::PoseStamped> history_;
        std::unique_ptr<rviz::BillboardLine> lines_;

        rviz::RosTopicProperty* property_topic_;
        rviz::BoolProperty*     property_line_view_;
        rviz::FloatProperty*    property_line_width_;
        rviz::ColorProperty*    property_line_color_;
};

}

#endif
