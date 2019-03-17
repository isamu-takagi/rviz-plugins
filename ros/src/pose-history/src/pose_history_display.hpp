#ifndef POSE_HISTORY_HPP
#define POSE_HISTORY_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
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

        std::deque<geometry_msgs::PoseStamped> history_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        std::unique_ptr<rviz::RosTopicProperty> topic_property_;
        std::unique_ptr<rviz::BillboardLine> lines_;
};

}

#endif
