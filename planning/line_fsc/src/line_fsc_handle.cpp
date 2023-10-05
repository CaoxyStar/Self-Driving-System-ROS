#include "../include/line_fsc/line_fsc_handle.hpp"

namespace ns_line_fsc {

LineFSC_Handle::LineFSC_Handle(ros::NodeHandle &nodeHandle) :
        nodeHandle_(nodeHandle), line_fsc_() {
                ROS_INFO("Constructing LineFSC Handle.");
                LoadParameters();
                subscribeToTopics();
                publishToTopics();

                this->task_status.status = tr23_msg::InnerASStatus::OFF;
                this->map_status.status = tr23_msg::PlannerStatus::DISABLED;
                this->map_status.task_finished = false;
                this->current_event.status = tr23_msg::Event::IDLE;
        }

void LineFSC_Handle::LoadParameters() {
        ROS_INFO("Loading LineFSC Handle parameters.");
        if (!nodeHandle_.param<float>("k_e", line_fsc_.k_e, 1)) { ROS_WARN_STREAM("Did not load k_e. Standard value is: " << line_fsc_.k_e); }
        if (!nodeHandle_.param<float>("k_v", line_fsc_.k_v, 0.00001)) { ROS_WARN_STREAM("Did not load k_v. Standard value is: " << line_fsc_.k_v); }
        if (!nodeHandle_.param<float>("vel_limit", line_fsc_.vel_limit, 1)) { ROS_WARN_STREAM("Did not load vel_limit. Standard value is: " << line_fsc_.vel_limit); }

        if (!nodeHandle_.param<std::string>("topicName/perception_topicName", this->perception_topicName, "/perception/lidar_cluster")) { ROS_WARN_STREAM("Did not load perception_topicName. Standard value is: " << this->perception_topicName); }
        if (!nodeHandle_.param<std::string>("topicName/perception_finish_topicName", this->perception_finish_topicName, "/perception/lidar_cluster/finish_line")) { ROS_WARN_STREAM("Did not load perception_finish_topicName. Standard value is: " << this->perception_finish_topicName); }
        if (!nodeHandle_.param<std::string>("topicName/vel_topicName", this->vel_topicName, "/can0/vehicle_state")) { ROS_WARN_STREAM("Did not load vel_topicName. Standard value is: " << this->vel_topicName); }
        if (!nodeHandle_.param<std::string>("topicName/state_topicName", this->state_topicName, "/guardian/inner_as_status")) { ROS_WARN_STREAM("Did not load state_topicName. Standard value is: " << this->state_topicName); }
        if (!nodeHandle_.param<std::string>("topicName/planningState_topicName", this->planningState_topicName, "/planner/planner_status")) { ROS_WARN_STREAM("Did not load planningState_topicName. Standard value is: " << this->planningState_topicName); }
        if (!nodeHandle_.param<std::string>("topicName/angle_topicName", this->angle_topicName, "/control_angle_aim")) { ROS_WARN_STREAM("Did not load angle_topicName. Standard value is: " << this->angle_topicName); }
        if (!nodeHandle_.param<std::string>("topicName/event_topicName", this->event_topicName, "/guardian/current_event")) { ROS_WARN_STREAM("Did not load event_topicName. Standard value is: " << this->event_topicName); }
}

void LineFSC_Handle::run() {
        if((this->task_status.status == tr23_msg::InnerASStatus::STAND_BY || this->task_status.status == tr23_msg::InnerASStatus::STARTING || 
                this->task_status.status == tr23_msg::InnerASStatus::OPERATING || this->task_status.status == tr23_msg::InnerASStatus::TASK_FINISHED ) 
                && (this->current_event.status == tr23_msg::Event::ACCELERATION)) {
                        this->line_fsc_.runAlgorithm();
        } else {
                this->line_fsc_.run_flag = false;
                this->map_status.status = tr23_msg::PlannerStatus::DISABLED;
        }
        if(this->line_fsc_.run_flag) {
                this->map_status.status = tr23_msg::PlannerStatus::MAP_OK;
                this->angle_msg.steeringWheelAngleAim = this->line_fsc_.getDelta();
                this->angle_publisher_.publish(this->angle_msg);
        }
        this->plan_publisher_.publish(this->map_status);
}

void LineFSC_Handle::subscribeToTopics() {
        this->cluster_subscriber_ = nodeHandle_.subscribe(this->perception_topicName, 1, &LineFSC_Handle::cluster_subscriberCallback, this);
        this->vel_subscriber_ = nodeHandle_.subscribe(this->vel_topicName, 1, &LineFSC_Handle::vel_subscriberCallback, this);
        this->state_subscriber_ = nodeHandle_.subscribe(this->state_topicName, 1, &LineFSC_Handle::state_subscriberCallback, this);
        this->finish_line_subscriber_ = nodeHandle_.subscribe(this->perception_finish_topicName, 1, &LineFSC_Handle::finish_subscribeCallback, this);
        this->event_subscriber_ = nodeHandle_.subscribe(this->event_topicName, 1, &LineFSC_Handle::event_subscribeCallback, this);
}

void LineFSC_Handle::publishToTopics() {
        this->plan_publisher_= nodeHandle_.advertise<tr23_msg::PlannerStatus>(this->planningState_topicName, 5);
        this->angle_publisher_ = nodeHandle_.advertise<tr23_msg::LateralControl>(this->angle_topicName, 5);
}

void LineFSC_Handle::cluster_subscriberCallback(const sensor_msgs::PointCloud &msg) {
        this->line_fsc_.setPerceptionData(msg);
}

void LineFSC_Handle::vel_subscriberCallback(const tr23_msg::VehicleState &msg) {
        this->line_fsc_.setVelocity(msg.vehicleSpeed);
}

void LineFSC_Handle::state_subscriberCallback(const tr23_msg::InnerASStatus &msg) {
        this->task_status.status = msg.status;
}

void LineFSC_Handle::finish_subscribeCallback(const sensor_msgs::PointCloud &msg) {
        double distance = 0;
        for(int i = 0; i<msg.points.size(); i++)
        {
                distance += msg.points[i].x;
        }
        distance = distance / msg.points.size();
        if(distance <= 5){
                this->map_status.task_finished = true;
        } 
}

void LineFSC_Handle::event_subscribeCallback(const tr23_msg::Event &msg) {
        this->current_event.status = msg.status;
}

}