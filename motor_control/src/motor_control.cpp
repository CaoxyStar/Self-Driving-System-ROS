#include "../include/motor_control/motor_control.hpp"

namespace ns_motorControl {

MotorControl::MotorControl(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
        ROS_INFO("Constructing MotorControl Handle.");
        this->can_msg_initialize();
        this->subscribeToTopics();
        this->publishToTopics();
        this->ClutchMotor_PositionCorrection();
        this->vehicleStatus.status = tr23_msg::InnerASStatus::OFF;
}

void MotorControl::run() {
        this->Datalog_pub();
        switch(this->current_event.status) {
                case tr23_msg::Event::ACCELERATION:
                case tr23_msg::Event::EBSTEST:
                        switch(this->vehicleStatus.status)
                        {
                                case tr23_msg::InnerASStatus::STARTING:
                                        this->NormalMode();
                                        break;
                                case tr23_msg::InnerASStatus::OPERATING:
                                        this->NormalMode();
                                        break;
                                case tr23_msg::InnerASStatus::EMERGENCY_STOP:
                                        this->EmergencyMode();
                                        break;
                        }
                case tr23_msg::Event::INSPECTION:
                        this->TestMode();
                        break;             
        }
        
}

void MotorControl::publishToTopics() {
        this->motor_publisher_ = this->nodeHandle_.advertise<can_msgs::Frame>("can0/sent_messages", 5);
        this->datalogger_braking_publisher_ = this->nodeHandle_.advertise<tr23_msg::BrakingControl>("/result_braking_aim", 5);
}

void MotorControl::subscribeToTopics() {
        this->lateralControl_subscriber_ = this->nodeHandle_.subscribe("/control_angle_aim", 5, &MotorControl::angleControlCallback, this);
        this->engineControl_subscriber_ = this->nodeHandle_.subscribe("/controller/long_controller/engine_control_aim", 5, &MotorControl::engineControlCallback, this);
        this->brakingControl_ecu_subscriber_ = this->nodeHandle_.subscribe("/brakingControl_ecu_aim", 5, &MotorControl::ecu_brakingControlCallback, this);
        this->brakingControl_ros_subscriber_ = this->nodeHandle_.subscribe("/brakingControl_ros_aim", 5, &MotorControl::ros_brakingControlCallback, this);
        this->vehicleStatus_subscriber_ = this->nodeHandle_.subscribe("/guardian/inner_as_status", 5, &MotorControl::vehicleStateCallback, this);
        this->event_subscriber_ = this->nodeHandle_.subscribe("/guardian/current_event", 5, &MotorControl::eventCallback, this);
}

void MotorControl::Datalog_pub() {
        tr23_msg::BrakingControl msg;
        if(this->brakingAim_ecu > this->brakingAim_ros) {
                msg.brakingAim = this->brakingAim_ecu;
        } else {
                msg.brakingAim = this->brakingAim_ros;
        }
        this->datalogger_braking_publisher_.publish(msg);
}

void MotorControl::NormalMode()
{
        if(this->get_LateralAim) {
                this->SteeringMotor_pub();
        }
        if(this->get_LongitudinalAim) {
                this->ClutchMotor_pub();
                this->Throttle_pub();
        }
        if(this->get_BrakingAim) {
                this->BrakingMotor_pub();
        }
}

void MotorControl::TestMode()
{
        if(this->get_LongitudinalAim) {
                this->ClutchMotor_pub();
                this->Throttle_pub();
        }
        static int32_t angle = 0;
        static bool inverse = false;
        int32_t angle_input = angle*890;
        this->Steering_TestMode_msg.data[4] = *(uint8_t*)(&angle_input);
        this->Steering_TestMode_msg.data[5] = *((uint8_t*)(&angle_input)+1);
        this->Steering_TestMode_msg.data[6] = *((uint8_t*)(&angle_input)+2);
        this->Steering_TestMode_msg.data[7] = *((uint8_t*)(&angle_input)+3);
        for(int i = 0; i < 10; i++) {
                this->motor_publisher_.publish(this->Steering_TestMode_msg);
        }
        if(!inverse) {
                angle++;
                if(angle == 45) {
                        inverse = true;
                }
        } else {
                angle--;
                if(angle == -45) {
                        inverse = false;
                }
        }
        //ROS_INFO("angele: %d", angle);
}

void MotorControl::EmergencyMode()
{
        if(this->get_LateralAim) {
                this->SteeringMotor_pub();
        }
        if(this->get_LongitudinalAim) {
                this->ClutchMotor_pub();
                this->Throttle_pub();
        }
        if(this->get_BrakingAim) {
                this->BrakingMotor_pub();
        }
}

void MotorControl::vehicleStateCallback(const tr23_msg::InnerASStatus &msg)
{
        this->vehicleStatus.status = msg.status;
}

void MotorControl::angleControlCallback(const tr23_msg::LateralControl &msg)
{
        this->get_LateralAim = true;
        this->angleAim = (int32_t)(msg.steeringWheelAngleAim*890);
}

void MotorControl::engineControlCallback(const tr23_msg::EngineControlAim &msg)
{
        this->get_LongitudinalAim = true;
        this->clutchAim = (int32_t)(msg.clutchAim*890);
        this->throttleAim = (uint8_t)(msg.throttleAim / 0.4);
        this->gearAim = (uint8_t)msg.gearShiftRequest;
}

void MotorControl::ecu_brakingControlCallback(const tr23_msg::BrakingControl &msg)
{
        this->get_BrakingAim = true;
        this->brakingAim_ecu = msg.brakingAim;
}

void MotorControl::ros_brakingControlCallback(const tr23_msg::BrakingControl &msg)
{
        this->get_BrakingAim = true;
        this->brakingAim_ros = msg.brakingAim;
}

void MotorControl::eventCallback(const tr23_msg::Event &msg) {
        this->current_event.status = msg.status;
}

void MotorControl::SteeringMotor_pub() {
        this->SteeringControl_msg.data[4] = *(uint8_t*)(&(this->angleAim));
        this->SteeringControl_msg.data[5] = *(((uint8_t*)(&this->angleAim))+1);
        this->SteeringControl_msg.data[6] = *(((uint8_t*)(&this->angleAim))+2);
        this->SteeringControl_msg.data[7] = *(((uint8_t*)(&this->angleAim))+3);
        this->motor_publisher_.publish(this->SteeringControl_msg);
}

void MotorControl::ClutchMotor_PositionCorrection() {
        this->motor_publisher_.publish(this->ClutchTorqueControl_msg);
        sleep(1);
        this->motor_publisher_.publish(this->ClutchSetControl_msg);
}

void MotorControl::ClutchMotor_pub() {
        this->ClutchAngleControl_msg.data[4] = *(uint8_t*)(&(this->clutchAim));
        this->ClutchAngleControl_msg.data[5] = *(((uint8_t*)(&this->clutchAim))+1);
        this->ClutchAngleControl_msg.data[6] = *(((uint8_t*)(&this->clutchAim))+2);
        this->ClutchAngleControl_msg.data[7] = *(((uint8_t*)(&this->clutchAim))+3);
        this->motor_publisher_.publish(this->ClutchAngleControl_msg);
}

void MotorControl::Throttle_pub() {
        this->ThrottleControl_msg.data[0] = this->throttleAim;
        this->ThrottleControl_msg.data[2] = this->gearAim;
        this->motor_publisher_.publish(this->ThrottleControl_msg);
}

void MotorControl::BrakingMotor_pub() {
        int16_t brakingAim_MAX;
        if(this->brakingAim_ecu > this->brakingAim_ros) {
                brakingAim_MAX = (int16_t)(this->brakingAim_ecu*20);
        } else {
                brakingAim_MAX = (int16_t)(this->brakingAim_ros*20);
        }
        this->BrakingControl_msg.data[4] = *(uint8_t*)(&brakingAim_MAX);
        this->BrakingControl_msg.data[5] = *((uint8_t*)(&brakingAim_MAX)+1);
        this->motor_publisher_.publish(this->BrakingControl_msg);
}

void MotorControl::can_msg_initialize() {
        // initialize steering motor msg
        this->SteeringControl_msg.id = 0x141;
        this->SteeringControl_msg.dlc = 8;
        this->SteeringControl_msg.data[0] = 0xA4;
        this->SteeringControl_msg.data[1] = 0;
        this->SteeringControl_msg.data[2] = 0;
        this->SteeringControl_msg.data[3] = 4;
        // initialize steering motor TestMode msg
        this->Steering_TestMode_msg.id = 0x141;
        this->Steering_TestMode_msg.dlc = 8;
        this->Steering_TestMode_msg.data[0] = 0xA4;
        this->Steering_TestMode_msg.data[1] = 0;
        this->Steering_TestMode_msg.data[2] = 0;
        this->Steering_TestMode_msg.data[3] = 4;
        // initialize clutch motor angle_control msg
        this->ClutchAngleControl_msg.id = 0x142;
        this->ClutchAngleControl_msg.dlc = 8;
        this->ClutchAngleControl_msg.data[0] = 0xA4;
        this->ClutchAngleControl_msg.data[1] = 0;
        this->ClutchAngleControl_msg.data[2] = 0;
        this->ClutchAngleControl_msg.data[3] = 4;
        // initialize clutch motor torque_control msg
        int16_t torque = 500;
        this->ClutchTorqueControl_msg.id = 0x142;
        this->ClutchTorqueControl_msg.dlc = 8;
        this->ClutchTorqueControl_msg.data[0] = 0xA1;
        this->ClutchTorqueControl_msg.data[1] = 0x00;
        this->ClutchTorqueControl_msg.data[2] = 0x00;
        this->ClutchTorqueControl_msg.data[3] = 0x00;
        this->ClutchTorqueControl_msg.data[4] = *(uint8_t*)(&torque);
        this->ClutchTorqueControl_msg.data[5] = *((uint8_t*)(&torque)+1);
        this->ClutchTorqueControl_msg.data[6] = 0x00;
        this->ClutchTorqueControl_msg.data[7] = 0x00;
        // initialize clutch motor set0_control msg
        this->ClutchSetControl_msg.id = 0x142;
        this->ClutchSetControl_msg.dlc = 8;
        this->ClutchSetControl_msg.data[0] = 0x19;
        this->ClutchSetControl_msg.data[1] = 0x00;
        this->ClutchSetControl_msg.data[2] = 0x00;
        this->ClutchSetControl_msg.data[3] = 0x00;
        this->ClutchSetControl_msg.data[4] = 0x00;
        this->ClutchSetControl_msg.data[5] = 0x00;
        this->ClutchSetControl_msg.data[6] = 0x00;
        this->ClutchSetControl_msg.data[7] = 0x00;
        // initialize braking motor msg
        this->BrakingControl_msg.id = 0x143;
        this->BrakingControl_msg.dlc = 8;
        this->BrakingControl_msg.data[0] = 0xA1;
        this->BrakingControl_msg.data[1] = 0x00;
        this->BrakingControl_msg.data[2] = 0x00;
        this->BrakingControl_msg.data[3] = 0x00;
        this->BrakingControl_msg.data[6] = 0x00;
        this->BrakingControl_msg.data[7] = 0x00;
        // initialize throttle control msg
        this->ThrottleControl_msg.id = 0x132;
        this->ThrottleControl_msg.dlc = 8;
}

}
