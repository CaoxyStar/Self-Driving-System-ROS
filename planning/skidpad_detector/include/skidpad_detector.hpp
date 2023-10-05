#ifndef SKIDPAD_DETECTOR_HPP
#define SKIDPAD_DETECTOR_HPP

#include "tr23_msg/Trajectory.h"
#include "tr23_msg/PlannerStatus.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "pcl/io/pcd_io.h"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include "fstream"
#include "cmath"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <mutex>

namespace ns_skidpad_detector {

class SkidpadDetector {

 public:
  // Constructor
  SkidpadDetector(ros::NodeHandle& nh);

	// Getters
  nav_msgs::Path getTransPath();
  nav_msgs::Path getStandardPath();
  sensor_msgs::PointCloud getStandardCloud();
  tr23_msg::Trajectory getPath();

	// Setters
  void setclusterFiltered(sensor_msgs::PointCloud msg);

  // main
  void runAlgorithm();

  // flags
  bool getClusterFlag, matchFlag;

private:
  // main handle
	ros::NodeHandle& nh_;

  // params (file path, fixed value)
  std::string path_pcd_, path_x_, path_y_, path_vel_, path_theta_;
  double start_length_, lidar2imu_, threshold_;

  // save cluster and standard_pointcloud
  sensor_msgs::PointCloud cluster, skidpad_map;

  // save paths
  nav_msgs::Path trans_path, standard_path;
  tr23_msg::Trajectory withVel_path;

  // initial functions
  void loadParameters();
  void loadFiles();
  
  void Pathcreate(Eigen::Matrix4f RT_Matrix);
};
}

#endif //SKIDPAD_DETECTOR_HPP
