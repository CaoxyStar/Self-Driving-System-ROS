#ifndef LINE_FSC_HPP
#define LINE_FSC_HPP

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <tr23_msg/VehicleState.h>
#include <tr23_msg/InnerASStatus.h>
#include <tr23_msg/PlannerStatus.h>
#include <tr23_msg/LateralControl.h>
#include <tr23_msg/Event.h>
#include <cmath>
#include <ros/ros.h>


namespace ns_line_fsc {

class LineFSC {

public:
        void setPerceptionData(sensor_msgs::PointCloud msg);
        void setVelocity(float vel);
        float getDelta();
        void runAlgorithm();

        bool run_flag = false;
        float vel_limit, k_e, k_v;

private:
        void LinearFitting(sensor_msgs::PointCloud points, float &k, float &b);
        void ave(float k1, float b1, float k2, float b2, float &k, float &b);
        void distance(float k, float b, float x = 0, float y = 0);
        void stanley(float vel, float theta,float e);

        sensor_msgs::PointCloud perception_data;
        float velocity;
        float k_left, b_left, k_right, b_right, k_lane, b_lane;
        float delta, dist;
        bool get_cloud_flag = false;
        bool get_vel_flag = false;
};

}

#endif