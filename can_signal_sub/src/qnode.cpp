/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <iostream>
#include <sstream>
#include "../include/can_signal_sub/qnode.hpp"
#include "tf/transform_datatypes.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

can_signal_sub_Handle::can_signal_sub_Handle(ros::NodeHandle &nodehandle)
{
    // can_signal_subscriber = nh.subscribe("/can0/received_messages", 1000, &can_signal_sub_Handle::canCallback, this);
    // engine_state_subscriber = nh.subscribe("/can0/received_messages", 1000, &can_signal_sub_Handle::engine_callback, this);
    // location_state_subscriber = nh.subscribe("/can0/received_messages", 1000, &can_signal_sub_Handle::location_callback, this);
    // this->subscriber = nodehandle.subscribe("/can0/received_messages", 1000, &can_signal_sub_Handle::callbackFcn, this);
    // this->LongitudinalControl_subscriber = nodehandle.subscribe("/control/longitudinal", 1000, &can_signal_sub_Handle::LongitudinalControl_BackFcn, this);
    // this->LateralControl_subscriber = nodehandle.subscribe("/control/lateral", 1000, &can_signal_sub_Handle::LateralControl_BackFcn, this);
    // this->imu_publisher = nodehandle.advertise<sensor_msgs::Imu>("/Imu0", 1000);
    // this->gnss_publisher = nodehandle.advertise<sensor_msgs::NavSatFix>("/Gnss0", 1000);
    // this->control_publisher = nodehandle.advertise<can_msgs::Frame>("/can0/sent_messages", 1000);
    
    this->LineFSC_subscriber = nodehandle.subscribe("/can0/received_messages", 1, &can_signal_sub_Handle::LineFSC_BackFcn, this);
    this->LineFSC_velPublisher = nodehandle.advertise<tr23_msg::VehicleState>("/can0/vehicle_state", 1);
    this->LineFSC_engineSpeedPublisher = nodehandle.advertise<tr23_msg::EngineState>("/can0/engine_state", 1);
}

void can_signal_sub_Handle::LineFSC_BackFcn(const can_msgs::Frame::ConstPtr& msg) {
    if(msg->id == 1600) {
        tr23_msg::VehicleState state;
        state.vehicleSpeed = (float)LineFSC_signal_list[0].compute_signal(msg);
        LineFSC_velPublisher.publish(state);
    }
    if(msg->id == 0x64A){
        tr23_msg::EngineState engineState;
        engineState.engineSpd = (int16_t)LineFSC_signal_list[1].compute_signal(msg);
        LineFSC_engineSpeedPublisher.publish(engineState);
    }
}

void can_signal_sub_Handle::LongitudinalControl_BackFcn(const tr23_msg::LongitudinalControl::ConstPtr& msg)
{
    this->Longitudinal_control_Frame.id = 306;
    uint64_t brakinAim = this->LGDcontrol_signal_list[0].convert_motorola_signal((float)msg->brakingAim);
    uint64_t gearShifRequest = this->LGDcontrol_signal_list[1].convert_motorola_signal((float)msg->gearShiftRequest);
    uint64_t throttleAim = this->LGDcontrol_signal_list[2].convert_motorola_signal((float)msg->throttleAim);
    uint64_t clutchAim = this->LGDcontrol_signal_list[3].convert_motorola_signal((float)msg->clutchAim);
    uint64_t result = brakinAim + gearShifRequest + throttleAim + clutchAim;
    for(int i =0;i<8;i++){
        uint8_t temp = (uint8_t)((result<<(i*8))>>56);
        this->Longitudinal_control_Frame.data[i] = temp;
    }
}

void can_signal_sub_Handle::LateralControl_BackFcn(const tr23_msg::LateralControl::ConstPtr& msg)
{
    this->Lateral_control_Frame.id = 305;
    uint64_t steeringAngle = this->Lateralcontrol_signal_list[0].convert_motorola_signal((float)msg->steeringWheelAngleAim);
    for(int i =0;i<8;i++){
        uint8_t temp = (uint8_t)((steeringAngle<<(i*8))>>56);
        this->Lateral_control_Frame.data[i] = temp;
    }
}


void can_signal_sub_Handle::callbackFcn(const can_msgs::Frame::ConstPtr& msg)
{
    for(int i = 0;i < (this->test_signal_list.size());i++)
    {
        if(msg->id!=this->test_signal_list[i].ID)
            continue;
        else
        {
            // ROS_INFO("%d", msg->id);
            double result = test_signal_list[i].compute_signal(msg);
            
            if(test_signal_list[i].signal_name == "AccelVehicleX_INS")
            {
                this->imuFrame.linear_acceleration.x = result;
            }
            else if(test_signal_list[i].signal_name == "AccelVehicleY_INS")
            {
                this->imuFrame.linear_acceleration.y = result;
            }
            else if(test_signal_list[i].signal_name == "AccelVehicleZ_INS")
            {
                this->imuFrame.linear_acceleration.z = result;
            }
            else if(test_signal_list[i].signal_name == "AngRateVehicleX_INS")
            {
                this->imuFrame.angular_velocity.x = result;
            }
            else if(test_signal_list[i].signal_name == "AngRateVehicleY_INS")
            {
                this->imuFrame.angular_velocity.y = result;
            }
            else if(test_signal_list[i].signal_name == "AngRateVehicleZ_INS")
            {
                this->imuFrame.angular_velocity.z = result;
            }
            else if(test_signal_list[i].signal_name == "AngleHeading_INS")
            {
                this->angularPos.yaw = result;
            }
            else if(test_signal_list[i].signal_name == "AnglePitch_INS")
            {
                this->angularPos.pitch = result;
            }
            else if(test_signal_list[i].signal_name == "AngleRoll_INS")
            {
                this->angularPos.roll = result;
            }
            else if(test_signal_list[i].signal_name == "GnssLat_GNSS")
            {
                this->gnssFrame.latitude = result;
            }
            else if(test_signal_list[i].signal_name == "GnssLon_GNSS")
            {
                this->gnssFrame.longitude = result;
            }
            else if(test_signal_list[i].signal_name == "GnssAlt_GNSS")
            {
                this->gnssFrame.altitude = result;
            }
        }
    }
}



double can_signal::compute_signal(const can_msgs::Frame::ConstPtr& msg)
{
    if (this->message_format == "Intel")
        return this->compute_intel_signal(msg);
    else
        return this->compute_motorola_signal(msg);
}

uint64_t can_signal::convert_signal(float value) {
    if (this->message_format == "Intel")
        return this->convert_intel_signal(value);
    else
        return this->convert_motorola_signal(value);
}

uint64_t can_signal::convert_intel_signal(float value)
{
    uint64_t result_ = (uint64_t)((value - this->bias)/this->factor);
    uint64_t result = ((result_ << (64-this->signal_length)) >> this->start_bit);
    return result;
}

uint64_t can_signal::convert_motorola_signal(float value)
{
    uint64_t value_ = (uint64_t)((value - this->bias)/this->factor);
    uint64_t result_ = 0;
    for(int i = 1 ;i<=(this->signal_length/8);i++){
        result_ += (((value_ << (64-i*8)) >> 56) << ((this->signal_length/8-i)*8));
    }
    uint64_t result = (result_ << (64 - this->signal_length + this->start_byte*8));
    return result;
}

double can_signal::compute_intel_signal(const can_msgs::Frame::ConstPtr& msg)
{
    uint64_t value = 0;
    for(uint8_t i = 0;i < this->message_length;i++)
        value += (uint64_t)(msg->data[i]) << (i*8);
    uint8_t left_move = this->message_length * 8 - this->start_bit - this->signal_length;
    uint8_t right_move = this->start_bit + left_move;
    value = (value << left_move) >> right_move;

    double result = 0.0;
    if(((uint64_t)1 << (this->signal_length - 1)) & value)
    {
        value = value | this->mask;
        result =  (double)((int64_t)value) * this->factor + this->bias;
    }
    else
    {
        result = (double)value * this->factor + this->bias;
    }
    
    return result;
}

double can_signal::compute_motorola_signal(const can_msgs::Frame::ConstPtr& msg)
{
    uint64_t value = 0;
    for(uint8_t i = 0;i < this->message_length;i++)
        value += (uint64_t)(msg->data[i]) << ((this->message_length - 1 - i)*8);
    uint8_t msb = (this->message_length - 1 - this->start_byte) * 8 + (this->start_bit%8);
    uint8_t lsb = msb - this->signal_length + 1;
    uint8_t left_move = this->message_length * 8 - 1 - msb;
    uint8_t right_move = lsb + left_move;
    value = (value << left_move) >> right_move;

    double result = 0.0;
    if(((uint64_t)1 << (this->signal_length - 1)) & value)
    {
        value = value | this->mask;
        result =  (double)((int64_t)value) * this->factor + this->bias;
    }
    else
    {
        result = (double)value * this->factor + this->bias;
    }
    return result;

}

// float compute_intel_signal(const can_msgs::Frame::ConstPtr& msg, can_signal signal)
// {
//     uint64_t value = 0;
//     for(int i=0;i<8;i++)
//         value += (uint64_t)(msg->data[i]) << (i*8);
//     int left_move = signal.message_length * 8 - signal.start_bit - signal.signal_length;
//     int right_move = signal.start_bit + left_move;
//     value = (value << left_move) >> right_move;

//     float result = 0.0;
//     if(((uint64_t)1 << (signal.signal_length - 1)) & value)
//     {
//         value = value | signal.mask;
//         result =  (float)((int64_t)value)*signal.factor+signal.bias;
//     }
//     else
//     {
//         result = (float)value*signal.factor+signal.bias;
//     }
    
//     return result;
// }

// float compute_motorola_signal(const can_msgs::Frame::ConstPtr& msg, can_signal signal)
// {
//     uint64_t value = 0;
//     for(int i = 0;i < signal.message_length;i++)
//         value += (uint64_t)(msg->data[i]) << ((7-i)*8);
//     int msb = (7-signal.start_byte)*8 + (signal.start_bit%8);
//     int lsb = msb - signal.signal_length +1;
//     int left_move = signal.message_length * 8 - 1 - msb;
//     int right_move = lsb + left_move;
//     value = (value << left_move) >> right_move;

//     float result = 0.0;
//     if(((uint64_t)1 << (signal.signal_length - 1)) & value)
//     {
//         value = value | signal.mask;
//         result =  (float)((int64_t)value)*signal.factor+signal.bias;
//     }
//     else
//     {
//         result = (float)value*signal.factor+signal.bias;
//     }
//     return result;
// }

void can_signal_sub_Handle::run_()
{
    // tr23_msg::Location location_msg = get_location();
    auto ori = tf::createQuaternionFromRPY(this->angularPos.roll, this->angularPos.pitch, this->angularPos.yaw);
    this->imuFrame.orientation.w = ori.getW();
    this->imuFrame.orientation.x = ori.getX();
    this->imuFrame.orientation.y = ori.getY();
    this->imuFrame.orientation.z = ori.getZ();
    // this->imu_publisher.publish(this->imuFrame);
    // //ROS_INFO("x:%.8f\ty:%.8f\t", this->gnssFrame.latitude,this->gnssFrame.longitude);
    // this->gnss_publisher.publish(this->gnssFrame);
    // this->control_publisher.publish(this->Longitudinal_control_Frame);
    // this->control_publisher.publish(this->Lateral_control_Frame);
}


// tr23_msg::Location can_signal_sub_Handle::get_location()
// {
//     tr23_msg::Location msg;
//     msg.imu_x = location_state_record[0];
//     msg.imu_y = location_state_record[1];
//     msg.imu_z = location_state_record[2];
//     msg.longitude = location_state_record[3];
//     msg.latitude = location_state_record[4];
//     return msg;
// }
