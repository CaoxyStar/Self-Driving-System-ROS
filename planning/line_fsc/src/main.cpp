#include <ros/ros.h>
#include "../include/line_fsc/line_fsc_handle.hpp"

using namespace ns_line_fsc;

int main(int argc, char **argv) {
        ros::init(argc, argv, "line_fsc");
        ros::NodeHandle nodeHandle("~");
        LineFSC_Handle myLineFSC_Handle(nodeHandle);
        ros::Rate loop_rate(50);
        while(ros::ok()) {
                myLineFSC_Handle.run();
                ros::spinOnce();
                loop_rate.sleep();
        }
}