#include <ros/ros.h>
#include "../include/skidpad_detector_handle.hpp"

namespace ns_skidpad_detector {

// 八字节点句柄构造函数，同时初始化成员skidpad_detector_
SkidpadDetectorHandle::SkidpadDetectorHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle), skidpad_detector_(nodeHandle) {
  ROS_INFO("Constructing Handle");

  loadParameters();
  subscribeToTopics();
  publishToTopics();
  getStandardPath();
}

// 获取节点频率
int SkidpadDetectorHandle::getNodeRate() const { return node_rate_; }

// 获取标准路径
void SkidpadDetectorHandle::getStandardPath() 
{ 
  standard_path_ = skidpad_detector_.getStandardPath(); 
  ROS_INFO("Get standardpath successfully, the size of poses is %d", standard_path_.poses.size());
}

// 获取话题名称
void SkidpadDetectorHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("cluster_filtered_topic_name",
                                      cluster_filtered_topic_name_,
                                      "/perception/lidar_cluster")) {
    ROS_WARN_STREAM("Did not load cluster_filtered_topic_name. Standard value is: " << cluster_filtered_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("vehicle_position_topic_name",
                                      vehicle_position_topic_name_,
                                      "/vehicle/position")) {
    ROS_WARN_STREAM("Did not load vehicle_position_topic_name. Standard value is: " << vehicle_position_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("skidpad_path_topic_name",
                                      skidpad_path_topic_name_,
                                      "/planning/global_path")) {
    ROS_WARN_STREAM("Did not load skidpad_path_topic_name. Standard value is: " << skidpad_path_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("status_topic_name",
                                      status_topic_name_,
                                      "/planning/task_status")) {
    ROS_WARN_STREAM("Did not load skidpad_path_topic_name. Standard value is: " << status_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 50)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
  if (!nodeHandle_.param("slice/rear_len", rear_len, 100)) {
    ROS_WARN_STREAM("Did not load slice_len. Standard value is: " << rear_len);
  }
  if (!nodeHandle_.param("slice/end_flag", end_flag, 100)) {
    ROS_WARN_STREAM("Did not load end_flag. Standard value is: " << end_flag);
  }
  if (!nodeHandle_.param("slice/front_len", front_len, 50)) {
    ROS_WARN_STREAM("Did not load front_len. Standard value is: " << front_len);
  }
}

// 创建订阅者
void SkidpadDetectorHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  clusterFilteredSubscriber_ = nodeHandle_.subscribe(cluster_filtered_topic_name_, 1, &SkidpadDetectorHandle::clusterFilteredCallback, this);
  vehiclePositionSubscriber_ = nodeHandle_.subscribe(vehicle_position_topic_name_, 1, &SkidpadDetectorHandle::vehiclePositionCallback, this);
}

// 创建发布者
void SkidpadDetectorHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  skidpadPathPublisher_ = nodeHandle_.advertise<tr23_msg::Trajectory>(skidpad_path_topic_name_, 1); //发布带速度的调整后的路线
  statusPublisher_ = nodeHandle_.advertise<tr23_msg::PlannerStatus>(status_topic_name_, 1); //发布规划任务状态

  standardPathPublisher_ = nodeHandle_.advertise<nav_msgs::Path>("skidpad_standardPath", 1);  //发布基准路线
  standardCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("skidpad_standardCloud", 1); //发布基准点云
  transPathPublisher_ = nodeHandle_.advertise<nav_msgs::Path>("skidpad_transPath", 1);  //发布调整过后的路线
}

void SkidpadDetectorHandle::run() {
	skidpad_detector_.runAlgorithm();
  sendStandardPath();
  sendTransPath();
  sendStandardCloud();
  sendMsg();
  sendStatus();
  if(!get_trans_ok && skidpad_detector_.matchFlag){
    trans_path_ = skidpad_detector_.getTransPath();
    withVel_path_ = skidpad_detector_.getPath();
    get_trans_ok = true;
  }
}

void SkidpadDetectorHandle::sendMsg() {
  if(!get_trans_ok || task_finished) { return; }

  if(get_current_position){
    //找到距离当前最近的点
    double temp = std::numeric_limits<double>::infinity();
    int i = last_point - front_len;
    if(i<0) { i=0; }
    for(i; i<(last_point + rear_len) && (i<trans_path_.poses.size()); i++) {
      double dist = std::hypot(current_point.pose.position.x - trans_path_.poses[i].pose.position.x, 
          current_point.pose.position.y - trans_path_.poses[i].pose.position.y);
      if(dist<temp){
        temp = dist;
      }
      else{
        last_point = i-1;
        break;
      }
    }
    //从该点开始截取slice_len长度的路径片段并发布
    path_slice_.Path.header.frame_id = "map";
    for(int i  = last_point;i<(last_point+rear_len);i++){
      path_slice_.Path.poses.push_back(trans_path_.poses[i]);
      path_slice_.velocity.push_back(withVel_path_.velocity[i]);
    }

    get_current_position = false;
  }
  skidpadPathPublisher_.publish(path_slice_);

  //检测是否到达路径末端，若是则任务完成
  if((last_point+end_flag)>trans_path_.poses.size()){
    task_finished = true;
  }
}

void SkidpadDetectorHandle::sendStatus() {
  tr23_msg::PlannerStatus vehicle_status;
  vehicle_status.status = vehicle_status.MAP_OK;
  if(task_finished){
    task_finished = true;
  }
  else{
    task_finished = false;
  }
  statusPublisher_.publish(vehicle_status);
}


void SkidpadDetectorHandle::sendStandardCloud() {
  standardCloudPublisher_.publish(skidpad_detector_.getStandardCloud());
}

void SkidpadDetectorHandle::sendStandardPath() {
  static int i = 0;
  standard_path_temp_.header.frame_id = "map";
  standard_path_temp_.poses.push_back(standard_path_.poses[i]);
  i++;
  if(i == standard_path_.poses.size()){
    i=0;
    standard_path_temp_.poses.clear();
  }
  standardPathPublisher_.publish(standard_path_temp_);
}

void SkidpadDetectorHandle::sendTransPath() {
  if(!get_trans_ok) { return; }
  static int k = 0;
  trans_path_temp_.header.frame_id = "map";
  trans_path_temp_.poses.push_back(trans_path_.poses[k]);
  k++;
  if(k == trans_path_.poses.size()){
    k=0;
    trans_path_temp_.poses.clear();
  }
  transPathPublisher_.publish(trans_path_temp_);
}

void SkidpadDetectorHandle::clusterFilteredCallback(const sensor_msgs::PointCloud& msg) {
  skidpad_detector_.setclusterFiltered(msg);
}

void SkidpadDetectorHandle::vehiclePositionCallback(const geometry_msgs::PoseStamped& position) {
  current_point = position;
  get_current_position = true;
}

}