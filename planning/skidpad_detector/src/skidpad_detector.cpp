#include <ros/ros.h>
#include "../include/skidpad_detector.hpp"
#include <sstream>
#include <iostream>

namespace ns_skidpad_detector {
// Constructor
SkidpadDetector::SkidpadDetector(ros::NodeHandle &nh) : nh_(nh) {
  getClusterFlag = false;
  matchFlag = false;
  loadParameters();
  loadFiles();
};

// Getters
nav_msgs::Path SkidpadDetector::getTransPath() { return trans_path; }
nav_msgs::Path SkidpadDetector::getStandardPath() { return standard_path; }
sensor_msgs::PointCloud SkidpadDetector::getStandardCloud() { return skidpad_map; }
tr23_msg::Trajectory SkidpadDetector::getPath() { return withVel_path; }

// Setters
void SkidpadDetector::setclusterFiltered(sensor_msgs::PointCloud msg) {
  cluster = msg;
  getClusterFlag = true;
}

// laod params
void SkidpadDetector::loadParameters() {
  ROS_INFO("loading parameters");
  if (!nh_.param<std::string>("path/skidpad_map",
                                      path_pcd_,
                                      "/home/xiaoyu/TJURacing_Autonomous_System/ros/src/planning/skidpad_detector/config/skidpad.pcd")) {
    ROS_WARN_STREAM("Did not load path/skidpad_map. Standard value is: " << path_pcd_);
  }
  
 	if (!nh_.param<std::string>("path/path_x",
                                      path_x_,
                                      "/home/xiaoyu/TJURacing_Autonomous_System/ros/src/planning/skidpad_detector/config/path_x.txt")) {
    ROS_WARN_STREAM("Did not load path/path_x. Standard value is: " << path_x_);
  }

  if (!nh_.param<std::string>("path/path_y",
                                      path_y_,
                                      "/home/xiaoyu/TJURacing_Autonomous_System/ros/src/planning/skidpad_detector/config/path_y.txt")) {
    ROS_WARN_STREAM("Did not load path/path_y. Standard value is: " << path_y_);
  }


  if (!nh_.param<std::string>("path/path_vel",
                                      path_vel_,
                                      "/home/xiaoyu/TJURacing_Autonomous_System/ros/src/planning/skidpad_detector/config/vel.txt")) {
    ROS_WARN_STREAM("Did not load path/path_vel. Standard value is: " << path_vel_);
  }

  if (!nh_.param<std::string>("path/path_theta",
                                      path_theta_,
                                      "/home/xiaoyu/TJURacing_Autonomous_System/ros/src/planning/skidpad_detector/config/theta.txt")) {
    ROS_WARN_STREAM("Did not load path/path_theta. Standard value is: " << path_theta_);
  }


	if (!nh_.param("length/start", start_length_, 15.0)) {
    ROS_WARN_STREAM("Did not load start_length. Standard value is: " << start_length_);
  }

	if (!nh_.param("length/lidar2imu", lidar2imu_, 1.87)) {
    ROS_WARN_STREAM("Did not load lidar2imu. Standard value is: " << lidar2imu_);
  }

  if (!nh_.param("length/threshold", threshold_, 1.5)) {
    ROS_WARN_STREAM("Did not load length/threshold. Standard value is: " << threshold_);
  }
}

// load files
void SkidpadDetector::loadFiles() {
  // load pcd skidpad map
  pcl::PointCloud<pcl::PointXYZ> source_cloud;
	geometry_msgs::Point32 tmp_cloud;
	pcl::io::loadPCDFile (path_pcd_,source_cloud);

  // The front is the x-axis, and the left is the y-axis
	for(int i = 0; i < source_cloud.points.size(); i++)
	{
		tmp_cloud.x = source_cloud.points[i].y + start_length_ + lidar2imu_;
		tmp_cloud.y = -source_cloud.points[i].x;
		skidpad_map.points.push_back(tmp_cloud);
	}
  skidpad_map.header.frame_id = "map";

  // load skidpad path
  std::ifstream infile_x,infile_y,infile_vel,infile_theta;
	infile_x.open(path_x_);
	infile_y.open(path_y_);

  // load velocity and theta
  infile_vel.open(path_vel_);
  infile_theta.open(path_theta_);

	float path_x,path_y,path_vel,path_theta,test;

	while(!infile_x.eof() && !infile_y.eof() && !infile_vel.eof() && !infile_theta.eof())
	{
		infile_x>>path_x;
		infile_y>>path_y;
    infile_vel>>path_vel;
    infile_theta>>path_theta;
    
    // read path
		geometry_msgs::PoseStamped temp;
		temp.pose.position.x = path_x + lidar2imu_;
		temp.pose.position.y = path_y;

    // read velocity and add into Trajectory directly
    withVel_path.velocity.push_back(path_vel);

    //将角度转化为四元数形式保存
    Eigen::AngleAxisd rotation_vector(path_theta, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    Eigen::Vector4d orientation = q.coeffs().transpose();
    temp.pose.orientation.x = orientation[0];
    temp.pose.orientation.y = orientation[1];
    temp.pose.orientation.z = orientation[2];
    temp.pose.orientation.w = orientation[3];
    
    // set point frame
    temp.header.frame_id = "map";

    standard_path.poses.push_back(temp);
	}
  // set path frame
  standard_path.header.frame_id = "map";

  ROS_INFO("standard_path poses size : %d", standard_path.poses.size());

	infile_x.close();
	infile_y.close();
  infile_vel.close();
}

void SkidpadDetector::runAlgorithm() {
  // set cluster and not match
  if(!getClusterFlag || matchFlag)
    return;

  ROS_INFO("Start match!");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> Final;
  pcl::PointXYZ in_temp;
	pcl::PointXYZ out_temp;

  ROS_INFO("Skidpad_map size: %d", skidpad_map.points.size());
  ROS_INFO("Get cluster size: %d", cluster.points.size());

  // 将基准点云与扫到的点云进行匹配
  for(int i = 0; i < skidpad_map.points.size(); i++)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    int index = -1;
    for(int j = 0; j < cluster.points.size(); j++)
    {
      double dist = std::hypot(skidpad_map.points[i].x - cluster.points[j].x, skidpad_map.points[i].y - cluster.points[j].y);
      if(min_dist > dist) {
        min_dist = dist;
        index = j;
      }
    }
    ROS_INFO("Match standard_point %d and cluster_point %d, dist: %f", i, index, min_dist);

    //若配对点云距离超过threshold(1.5m)则舍弃，此处可调整threshold(1.5m)从而优化最终效果
    if(min_dist < threshold_) {
      in_temp.x = skidpad_map.points[i].x;
      in_temp.y = skidpad_map.points[i].y;
      in_temp.z = 0;
      cloud_in->points.push_back(in_temp);

      out_temp.x = cluster.points[index].x;
		  out_temp.y = cluster.points[index].y;
		  out_temp.z = 0;
      cloud_out->points.push_back(out_temp);
    }
  }
  ROS_INFO("Find %d points match!", cloud_out->points.size());

  // icp match
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  icp.setMaxCorrespondenceDistance(threshold_);  //该参数1.5和上方作用一样，将配对点距离超过1.5m的点对舍弃，其他三个参数一般不调
	icp.setTransformationEpsilon(1e-10); 
	icp.setEuclideanFitnessEpsilon(0.001); 
	icp.setMaximumIterations(1000);
  icp.align(Final);

  // 得到变换矩阵并返回
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	std::cout <<transformation<<std::endl;
	std::cout <<"------------------------------------------------\n";
  ROS_INFO("match ok!");
  matchFlag = true;
	Pathcreate(transformation);
}

void SkidpadDetector::Pathcreate(Eigen::Matrix4f RT_Matrix)
{
  //通过变换矩阵对基准路径上各点进行位姿变换，并将新位姿加入调整后的路径
	trans_path.poses.resize(standard_path.poses.size());
	for (int i = 0; i < standard_path.poses.size(); i++)
  {
    float pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,vel;
		pos_x = standard_path.poses[i].pose.position.x;
		pos_y = standard_path.poses[i].pose.position.y;
    pos_z = 0;
    ori_x = standard_path.poses[i].pose.orientation.x;
    ori_y = standard_path.poses[i].pose.orientation.y;
    ori_z = standard_path.poses[i].pose.orientation.z;
    ori_w = standard_path.poses[i].pose.orientation.w;

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.rotate(Eigen::Quaternionf(ori_w, ori_x, ori_y, ori_z));
    T.pretranslate(Eigen::Vector3f(pos_x, pos_y, pos_z));
		Eigen::Matrix4f result = RT_Matrix*T.matrix();
    
		trans_path.poses[i].header.frame_id = "map";
		trans_path.poses[i].header.stamp = ros::Time::now();
		trans_path.poses[i].pose.position.x = result(0, 3)/result(3, 3);
    trans_path.poses[i].pose.position.y = result(1, 3)/result(3, 3);
    trans_path.poses[i].pose.position.z = result(2, 3)/result(3, 3);
    Eigen::Matrix3f new_matrix;
    new_matrix << result(0, 0), result(0, 1), result(0, 2),
                  result(1, 0), result(1, 1), result(1, 2),
                  result(2, 0), result(2, 1), result(2, 2);
    Eigen::Quaternionf q = Eigen::Quaternionf(new_matrix);
    Eigen::Vector4f orientation = q.coeffs().transpose();
    trans_path.poses[i].pose.orientation.x = orientation[0];
    trans_path.poses[i].pose.orientation.y = orientation[1];
    trans_path.poses[i].pose.orientation.z = orientation[2];
    trans_path.poses[i].pose.orientation.w = orientation[3];

    withVel_path.Path = trans_path;
  }
}

}
