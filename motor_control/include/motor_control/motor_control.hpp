#ifndef  MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <unistd.h>
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <tr23_msg/LateralControl.h>
#include <tr23_msg/EngineControlAim.h>
#include <tr23_msg/BrakingControl.h>
#include <tr23_msg/InnerASStatus.h>
#include <tr23_msg/Event.h>

namespace ns_motorControl {

class MotorControl {

public:
        MotorControl(ros::NodeHandle &nodeHandle);
        void run();
        void subscribeToTopics();
        void publishToTopics();
        void can_msg_initialize();

        void SteeringMotor_pub();
        void ClutchMotor_pub();
        void ClutchMotor_PositionCorrection();
        void Throttle_pub();
        void BrakingMotor_pub();
        void Datalog_pub();

        void NormalMode();
        void TestMode();
        void EmergencyMode();

private:
        ros::NodeHandle nodeHandle_;
        ros::Publisher motor_publisher_;
        ros::Publisher datalogger_braking_publisher_;

        ros::Subscriber lateralControl_subscriber_;
        ros::Subscriber engineControl_subscriber_;
        ros::Subscriber brakingControl_ecu_subscriber_;
        ros::Subscriber brakingControl_ros_subscriber_;
        ros::Subscriber vehicleStatus_subscriber_;
        ros::Subscriber event_subscriber_;

        int32_t angleAim;
        int32_t clutchAim;
        uint8_t throttleAim;
        uint8_t gearAim;
        uint8_t brakingAim_ecu, brakingAim_ros;
        bool get_LateralAim = false;
        bool get_LongitudinalAim = false;
        bool get_BrakingAim = false;
        tr23_msg::InnerASStatus vehicleStatus;
        tr23_msg::Event current_event;

        can_msgs::Frame SteeringControl_msg, Steering_TestMode_msg, ThrottleControl_msg, BrakingControl_msg;
        can_msgs::Frame ClutchTorqueControl_msg, ClutchAngleControl_msg, ClutchSetControl_msg;

        void angleControlCallback(const tr23_msg::LateralControl &msg);
        void engineControlCallback(const tr23_msg::EngineControlAim &msg);
        void ros_brakingControlCallback(const tr23_msg::BrakingControl &msg);
        void ecu_brakingControlCallback(const tr23_msg::BrakingControl &msg);
        void vehicleStateCallback(const tr23_msg::InnerASStatus &msg);
        void eventCallback(const tr23_msg::Event &msg);
};

}

#endif