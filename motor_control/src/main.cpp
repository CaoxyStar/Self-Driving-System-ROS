#include "../include/motor_control/motor_control.hpp"

using namespace ns_motorControl;

int main(int argc, char** argv) {
        ros::init(argc,  argv, "motor_control") ;
        ros::NodeHandle nodeHandle;

        MotorControl myMotorControl_Handle(nodeHandle);
        ros::Rate loop_rate(50);
        while(ros::ok()) {
                myMotorControl_Handle.run();
                ros::spinOnce();
                loop_rate.sleep();
        }
}