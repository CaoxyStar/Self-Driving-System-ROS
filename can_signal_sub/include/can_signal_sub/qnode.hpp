#ifndef QNODE
#define QNODE
/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include<std_msgs/Int8.h>
#include<geometry_msgs/Twist.h>
#include<can_msgs/Frame.h>
#include<iostream>
#include"tr23_msg/Location.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <unordered_map>
#include"tr23_msg/LongitudinalControl.h"
#include"tr23_msg/LateralControl.h"
#include "tr23_msg/VehicleState.h"
#include "tr23_msg/EngineControlAim.h"
#include "tr23_msg/EngineState.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

class can_signal {
    public:
        std::string message_name;
        uint32_t ID;
        uint8_t message_length;
        std::string message_format;

        std::string signal_name;
        uint8_t start_byte;
        uint8_t start_bit;
        uint8_t signal_length;
        std::string data_type;
        double factor;
        double bias;
        double maximum;
        double minimum;
        std::string unit;
        uint64_t mask;
        
        double signal_value;
        double* target_value;

        uint8_t update_signal(double& value);
        double compute_signal(const can_msgs::Frame::ConstPtr& msg);
        uint64_t convert_signal(float value);
        uint64_t convert_intel_signal(float value);
        uint64_t convert_motorola_signal(float value);
        
    private:
        double compute_intel_signal(const can_msgs::Frame::ConstPtr& msg);
        double compute_motorola_signal(const can_msgs::Frame::ConstPtr& msg);
};


class can_signal_sub_Handle{
    public:
        can_signal_sub_Handle(ros::NodeHandle &nodehandle);  

        std::vector<can_signal> All_signal_list;

        std::vector<can_signal> engine_signal_list;
        std::vector<can_signal> test_signal_list;
        std::vector<can_signal> imu_signal_list;
        std::vector<can_signal> LGDcontrol_signal_list;
        std::vector<can_signal> Lateralcontrol_signal_list;
        std::vector<can_signal> LineFSC_signal_list;

        float location_state_record[5] = {0, 0, 0, 0, 0};

        struct{
            double yaw = 0;
            double pitch = 0;
            double roll = 0;
        } angularPos;
        sensor_msgs::Imu imuFrame;
        sensor_msgs::NavSatFix gnssFrame;

        //control_frame
        can_msgs::Frame Longitudinal_control_Frame;
        can_msgs::Frame Lateral_control_Frame;

        // std::unordered_map<uint32_t, can_signal> signalMap;
        // std::unordered_map<can_signal, float*> signalMap;
        
        void run_();

        enum LogLevel {
            Debug,
            Info,
            Warn,
            Error,
            Fatal
        };
        ros::Subscriber location_state_subscriber;
        ros::Subscriber subscriber;
        ros::Subscriber LongitudinalControl_subscriber;
        ros::Subscriber LateralControl_subscriber;

        ros::Subscriber LineFSC_subscriber;

        void callbackFcn(const can_msgs::Frame::ConstPtr& msg);
        void LongitudinalControl_BackFcn(const tr23_msg::LongitudinalControl::ConstPtr& msg);
        void LateralControl_BackFcn(const tr23_msg::LateralControl::ConstPtr& msg);
        void LineFSC_BackFcn(const can_msgs::Frame::ConstPtr& msg);

        ros::Publisher throttle_publisher;
        ros::Publisher imu_publisher;
        ros::Publisher gnss_publisher;
        ros::Publisher control_publisher;

        ros::Publisher LineFSC_velPublisher;
        ros::Publisher LineFSC_engineSpeedPublisher;

        tr23_msg::Location get_location();
};
 
//float compute_intel_signal(const can_msgs::Frame::ConstPtr& msg, can_signal signal);
//float compute_motorola_signal(const can_msgs::Frame::ConstPtr& msg, can_signal signal);
#endif
