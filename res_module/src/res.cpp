#include "../include/res_module/res.hpp"

namespace ns_res {

Res_Handle::Res_Handle(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
        ROS_INFO("Constructing RES Handle.");
        this->subscribeToTopics();
        this->publishToTopics();

        this->RES_Initialize();
        this->RES_Set();
        this->RES_State.status = tr23_msg::ResStatus::RES_STANDBY;
}

void Res_Handle::run() {
        this->state_publisher_.publish(this->RES_State);
}

void Res_Handle::subscribeToTopics() {
        this->res_subscriber_ = this->nodeHandle_.subscribe("/can0/received_messages", 5, &Res_Handle::resCallBack, this);
}

void Res_Handle::publishToTopics() {
        this->res_publisher_ = this->nodeHandle_.advertise<can_msgs::Frame>("/can0/sent_messages", 5);
        this->state_publisher_ = this->nodeHandle_.advertise<tr23_msg::ResStatus>("/res_status", 5);
}

// 待完善部分
void Res_Handle::resCallBack(const can_msgs::Frame &msg) {
        if(msg.id != (0x181)) { return; }
        switch(msg.data[0]){
            case 0:
                this->RES_State.status = tr23_msg::ResStatus::RES_EMERGENCY;
                ROS_INFO("EMERGENCY!");
                break;
            case 1:
                this->RES_State.status = tr23_msg::ResStatus::RES_STANDBY;
                ROS_INFO("OK STANDBY.");
                break;
            case 7:
            case 3:
                this->RES_State.status = tr23_msg::ResStatus::RES_GO;
                ROS_INFO("GO!");
                break;
        }
}

void Res_Handle::RES_Initialize()
{
    can_msgs::Frame initial_msg;
    initial_msg.id = 0x700 + 0x011;
    initial_msg.dlc = 1;
    initial_msg.data[0] = 0x00;
    for(int i =0;i<100;i++){
        this->res_publisher_.publish(initial_msg);
    }
}

void Res_Handle::RES_Checkup()
{
    can_msgs::Frame checkup_msg;
    checkup_msg.id = 0x000;
    checkup_msg.dlc = 2;
    checkup_msg.data[0] = 0x80;
    checkup_msg.data[1] = 0x011;
    this->res_publisher_.publish(checkup_msg);
}

void Res_Handle::RES_Set()
{
    can_msgs::Frame set_msg;
    set_msg.id = 0x000;
    set_msg.dlc = 2;
    set_msg.data[0] = 0x01;
    set_msg.data[1] = 0x011;
    for(int i =0 ;i<100; i++){
        this->res_publisher_.publish(set_msg);
    }
}

}