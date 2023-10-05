# include "../include/vehicle_communication/vehicle_communication.hpp"

namespace ns_VehicleCommunication {

VehicleCommunication::VehicleCommunication(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
        ROS_INFO("Constructing VehicleCommunication Handle.");
        this->subscribeToTopics();
        this->publishToTopics();

        this->DataLogger_0x500_msg.id = 0x500;
        this->DataLogger_0x500_msg.dlc = 8;

        this->DataLogger_0x502_msg.id = 0x502;
        this->DataLogger_0x502_msg.dlc = 5;

        this->ebs_monitor_state.status = tr23_msg::EBSMonitoringStatus::DISABLED;
}

void VehicleCommunication::subscribeToTopics() {
        this->can_Subscriber_ = nodeHandle_.subscribe("/can0/received_messages", 5, &VehicleCommunication::CanSubCallback, this);

        this->steeringAngle_Subscriber_ = nodeHandle_.subscribe("/control_angle_aim", 5, &VehicleCommunication::SteeringCallback, this);
        this->braking_Subscriber_ = nodeHandle_.subscribe("/result_braking_aim", 5, &VehicleCommunication::BrakingCallback, this);
        this->throttle_Subscriber_ = nodeHandle_.subscribe("/controller/long_controller/engine_control_aim", 5, &VehicleCommunication::ThrottleCallback, this);
        this->AS_state_Subscriber_ = nodeHandle_.subscribe("/guardian/as_status", 5, &VehicleCommunication::ASStateCallback, this);
        this->event_state_Subscriber_ = nodeHandle_.subscribe("/guardian/current_event", 5, &VehicleCommunication::EventStateCallback, this);
}

void VehicleCommunication::publishToTopics() {
        this->speed_Publisher_ = nodeHandle_.advertise<tr23_msg::VehicleState>("/can0/vehicle_state", 5);
        this->braking_Publisher_ = nodeHandle_.advertise<tr23_msg::BrakingControl>("/brakingControl_ecu_aim", 5);
        this->engine_speed_Publisher_ = nodeHandle_.advertise<tr23_msg::EngineState>("/can0/engine_state", 5);
        this->ebs_state_Publisher_ = nodeHandle_.advertise<tr23_msg::EBSMonitoringStatus>("/can0/ebs_monitoring_status", 5);

        this->can_Publisher_ = nodeHandle_.advertise<can_msgs::Frame>("/can0/sent_messages", 5);
}

void VehicleCommunication::run() {
        if(this->get_vel) { this->speed_Publisher_.publish(this->vehicle_speed); }
        if(this->get_braking) { this->braking_Publisher_.publish(this->braking_target); }
        if(this->get_engine) { this->engine_speed_Publisher_.publish(this->engine_state); };
        if(this->get_ebsState) { this->ebs_state_Publisher_.publish(this->ebs_monitor_state); }

        this->DataLogger_0x500_msg.data[0] = (uint8_t)this->vehicle_speed.vehicleSpeed;
        this->DataLogger_0x500_msg.data[1] = (uint8_t)this->vehicle_speed.vehicleSpeed;
        this->DataLogger_0x500_msg.data[2] = (uint8_t)this->steering_angle;
        this->DataLogger_0x500_msg.data[3] = (uint8_t)this->steering_angle;
        this->DataLogger_0x500_msg.data[4] = (uint8_t)this->result_braking;
        this->DataLogger_0x500_msg.data[5] = (uint8_t)this->result_braking;
        this->DataLogger_0x500_msg.data[6] = (uint8_t)this->throttle_aim;
        this->DataLogger_0x500_msg.data[7] = (uint8_t)this->throttle_aim;
        this->can_Publisher_.publish(this->DataLogger_0x500_msg);

        this->DataLogger_0x502_msg.data[0] = this->as_state + (this->ebs_state<< 3) + (this->task_state << 5);
        this->DataLogger_0x502_msg.data[1] = this->steering_state + (this->brake_state << 1) + (this->lap_counter << 3) + (this->cones_count << 7);
        this->DataLogger_0x502_msg.data[2] = (this->cones_count >> 1) + (this->cones_count << 7);
        this->DataLogger_0x502_msg.data[3] = (this->cones_count >> 1);
        this->DataLogger_0x502_msg.data[4] = 0;
        this->can_Publisher_.publish(this->DataLogger_0x502_msg);

        this->engine_speed_Publisher_.publish(this->engine_state);
        this->ebs_state_Publisher_.publish(this->ebs_monitor_state);
}

void VehicleCommunication::CanSubCallback(const can_msgs::Frame &msg)
{
        if(msg.id == 0x640) {
                this->get_vel = true;
                uint16_t speed = (((uint16_t)msg.data[4]) << 8) + (uint16_t)msg.data[5];
                this->vehicle_speed.vehicleSpeed = speed*0.1;
        } else if(msg.id == 0x739) {
                this->get_braking = true;
                uint8_t target = msg.data[2]*0.4;
                this->braking_target.brakingAim = target;
        } else if(msg.id == 0x64A) {
                this->get_engine = true;
                uint16_t engine_speed = (((uint16_t)msg.data[0]) << 8) + (uint16_t)msg.data[1];
                this->engine_state.engineSpd = engine_speed;
        } else if(msg.id == 0x740) {
                this->get_ebsState = true;
                this->ebs_monitor_state.status = msg.data[0];
                if ((msg.data[0] >= 6)&&(msg.data[0]<=12)) {
                        this->ebs_state = 1;
                } else if (msg.data[0] == 13) {
                        this->ebs_state = 2;
                } else {
                        this->ebs_state = 3;
                }
        } else {
                return;
        }
}

void VehicleCommunication::SteeringCallback(const tr23_msg::LateralControl &msg)
{
        this->steering_angle = msg.steeringWheelAngleAim;
}

void VehicleCommunication::BrakingCallback(const tr23_msg::BrakingControl &msg)
{
        this->result_braking = msg.brakingAim;
}

void VehicleCommunication::ThrottleCallback(const tr23_msg::EngineControlAim &msg)
{
        this->throttle_aim = msg.throttleAim;
}

void VehicleCommunication::ASStateCallback(const tr23_msg::ASStatus &msg)
{
        switch(msg.status){
                case tr23_msg::ASStatus::AS_OFF:
                        this->as_state = 1;
                        break;
                case tr23_msg::ASStatus::AS_INITCHECK:
                        this->as_state = 1;
                        break;
                case tr23_msg::ASStatus::AS_WAITING_FOR_EBS_CHECK:
                        this->as_state = 1;
                        break;
                case tr23_msg::ASStatus::AS_EBS_CHECKING:
                        this->as_state = 1;
                        break;
                case tr23_msg::ASStatus::AS_READY:
                        this->as_state = 2;
                        break;
                case tr23_msg::ASStatus::AS_READY_COOLDOWN:
                        this->as_state = 2;
                        break;
                case tr23_msg::ASStatus::AS_DRIVING:
                        this->as_state = 3;
                        break;
                case tr23_msg::ASStatus::AS_EMERGENCY:
                        this->as_state = 4;
                        break;
                case tr23_msg::ASStatus::AS_FINISHED:
                        this->as_state = 5;
                        break;
                default:
                        this->as_state = 1;
        }
}

void VehicleCommunication::EventStateCallback(const tr23_msg::Event &msg) {
        if(msg.status == tr23_msg::Event::ACCELERATION){
                this->task_state = 1;
        } else if (msg.status == tr23_msg::Event::SKIDPAD) {
                this->task_state = 2;
        } else if(msg.status == tr23_msg::Event::TRACKDRIVE) {
                this->task_state = 3;
        } else if(msg.status == tr23_msg::Event::EBSTEST) {
                this->task_state = 4;
        } else if(msg.status == tr23_msg::Event::INSPECTION) {
                this->task_state = 5;
        } else {
                this->task_state =0;
        }
}

}
