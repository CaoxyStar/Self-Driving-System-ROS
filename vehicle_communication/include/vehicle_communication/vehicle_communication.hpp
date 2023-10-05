#ifndef VEHICLE_COMMUNICATION_HPP
#define VEHICLE_COMMUNICATION_HPP

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <tr23_msg/VehicleState.h>
#include <tr23_msg/BrakingControl.h>
#include <tr23_msg/LateralControl.h>
#include <tr23_msg/EngineControlAim.h>
#include <tr23_msg/EngineState.h>
#include <tr23_msg/ASStatus.h>
#include <tr23_msg/EBSMonitoringStatus.h>
#include <tr23_msg/Event.h>

namespace ns_VehicleCommunication {

class VehicleCommunication {

public:
        VehicleCommunication(ros::NodeHandle &nodeHandle);
        void run();
        void subscribeToTopics();
        void publishToTopics();

private:
        ros::NodeHandle nodeHandle_;

        ros::Subscriber can_Subscriber_;
        // 0x500 subscriber
        ros::Subscriber steeringAngle_Subscriber_;
        ros::Subscriber throttle_Subscriber_;
        ros::Subscriber braking_Subscriber_;
        // 0x502 subscribers
        ros::Subscriber AS_state_Subscriber_;
        ros::Subscriber event_state_Subscriber_;
        // 发布者
        ros::Publisher can_Publisher_;
        ros::Publisher braking_Publisher_;
        ros::Publisher speed_Publisher_;
        ros::Publisher engine_speed_Publisher_;
        ros::Publisher ebs_state_Publisher_;
        
        // 待发布的rosmsg
        tr23_msg::VehicleState vehicle_speed;
        tr23_msg::EngineState engine_state;
        tr23_msg::BrakingControl braking_target;
        tr23_msg::EBSMonitoringStatus ebs_monitor_state;

        bool get_vel = false;
        bool get_braking = false;
        bool get_engine = false;
        bool get_ebsState = false;

        void CanSubCallback(const can_msgs::Frame &msg);

        // 0x500 msg
        can_msgs::Frame DataLogger_0x500_msg;
        // 0x500 members
        float steering_angle;
        float result_braking;
        float throttle_aim;
        // 0x500 callback
        void SteeringCallback(const tr23_msg::LateralControl &msg);
        void BrakingCallback(const tr23_msg::BrakingControl &msg);
        void ThrottleCallback(const tr23_msg::EngineControlAim &msg);

        // 0x502 msg
        can_msgs::Frame DataLogger_0x502_msg;
        // 0x502 members
        uint8_t as_state = 1;
        uint8_t task_state = 0;
        uint8_t ebs_state = 1;
        uint8_t steering_state = 1;
        uint8_t brake_state = 2;
        uint8_t lap_counter = 0;
        uint8_t cones_count = 8;
        // 0x502 callback
        void ASStateCallback(const tr23_msg::ASStatus &msg);
        void EventStateCallback(const tr23_msg::Event &msg);
};

}

#endif