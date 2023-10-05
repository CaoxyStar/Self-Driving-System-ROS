#ifndef LINE_FSC_HANDLE_HPP
#define LINE_FSC_HANDLE_HPP

#include <ros/ros.h>
#include "line_fsc.hpp"

namespace ns_line_fsc {

class LineFSC_Handle {

public:
        LineFSC_Handle(ros::NodeHandle &nodeHandle);
        void run();
        void subscribeToTopics();
        void publishToTopics();
        void LoadParameters();
        
private:
        LineFSC line_fsc_;

        ros::NodeHandle nodeHandle_;
        ros::Subscriber cluster_subscriber_;
        ros::Subscriber finish_line_subscriber_;
        ros::Subscriber vel_subscriber_;
        ros::Subscriber state_subscriber_;
        ros::Subscriber event_subscriber_;
        ros::Publisher plan_publisher_;
        ros::Publisher angle_publisher_;

        std::string perception_topicName, perception_finish_topicName, vel_topicName, state_topicName, planningState_topicName, angle_topicName, event_topicName;

        tr23_msg::InnerASStatus task_status;
        tr23_msg::PlannerStatus map_status;
        tr23_msg::LateralControl angle_msg;
        tr23_msg::Event current_event;

        void cluster_subscriberCallback(const sensor_msgs::PointCloud &msg);
        void finish_subscribeCallback(const sensor_msgs::PointCloud &msg);
        void vel_subscriberCallback(const tr23_msg::VehicleState &msg);
        void state_subscriberCallback(const tr23_msg::InnerASStatus &msg);
        void event_subscribeCallback(const tr23_msg::Event &msg);
};

}

#endif