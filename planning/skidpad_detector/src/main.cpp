#include <ros/ros.h>
#include "../include/skidpad_detector_handle.hpp"

typedef ns_skidpad_detector::SkidpadDetectorHandle SkidpadDetectorHandle;

int main(int argc, char **argv) {
    ros::init(argc, argv, "skidpad_detector");
    ros::NodeHandle nodeHandle("~");
    // 创建八字节点句柄
    SkidpadDetectorHandle mySkidpadDetectorHandle(nodeHandle);
    // 设置节点运行频率
    ros::Rate loop_rate(mySkidpadDetectorHandle.getNodeRate());
    while (ros::ok()) {
        mySkidpadDetectorHandle.run();
        ros::spinOnce();
        loop_rate.sleep();      
    }
    return 0;
}

