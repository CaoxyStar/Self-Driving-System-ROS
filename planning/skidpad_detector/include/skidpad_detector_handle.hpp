#ifndef SKIDPAD_DETECTOR_HANDLE_HPP
#define SKIDPAD_DETECTOR_HANDLE_HPP

#include "skidpad_detector.hpp"

namespace ns_skidpad_detector {

class SkidpadDetectorHandle {

 public:
  // Constructor
  SkidpadDetectorHandle(ros::NodeHandle &nodeHandle);

  // Getters
  int getNodeRate() const;

  // Initialize Function
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void getStandardPath();

  // main
  void run();

  // 消息发布函数
  void sendMsg();
  void sendStandardPath();
  void sendTransPath();
  void sendStandardCloud();
  void sendStatus();

  //切片发送路径
  tr23_msg::Trajectory path_slice_;
  geometry_msgs::PoseStamped current_point;
  bool get_current_position = false;
  bool task_finished = false;
  int last_point = 0;
  int rear_len;
  int end_flag;
  int front_len;

 private:
  //主节点
  ros::NodeHandle nodeHandle_;
  //主订阅者（收感知传来的点云）
  ros::Subscriber clusterFilteredSubscriber_;
  //订阅者(订阅车辆实时位置)
  ros::Subscriber vehiclePositionSubscriber_;
  //主发布者（将Trajectory传给控制）
  ros::Publisher skidpadPathPublisher_;
  //发布者（发布标准点云、标准路径、变换后无速度的路径，在rviz中显示）
  ros::Publisher standardPathPublisher_;
  ros::Publisher standardCloudPublisher_;
  ros::Publisher transPathPublisher_;
  //发布者（发布任务完成状态）
  ros::Publisher statusPublisher_;

  //CallBack Function
  void clusterFilteredCallback(const sensor_msgs::PointCloud& msg);
  void vehiclePositionCallback(const geometry_msgs::PoseStamped& position);

  //主订阅者和主发布者的话题名称
  std::string cluster_filtered_topic_name_;
  std::string skidpad_path_topic_name_;
  std::string vehicle_position_topic_name_;
  std::string status_topic_name_;

  //用于存储标准路径和规划好的路径，temp用于rviz显示
  nav_msgs::Path standard_path_;
  nav_msgs::Path standard_path_temp_;
  nav_msgs::Path trans_path_;
  nav_msgs::Path trans_path_temp_;
  bool get_trans_ok = false;

  //存储变换好带速度的路径
  tr23_msg::Trajectory withVel_path_;

  //节点频率
  int node_rate_;

  //八字规划对象
  SkidpadDetector skidpad_detector_;

};
}

#endif //SKIDPAD_DETECTOR_HANDLE_HPP
