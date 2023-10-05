#include "../include/res_module/res.hpp"

using namespace ns_res;

int main(int argc, char** argv) {
        ros::init(argc,  argv, "res") ;
        ros::NodeHandle nodeHandle;

        Res_Handle myRes_Handle(nodeHandle);
        ros::Rate loop_rate(50);
        while(ros::ok()) {
                myRes_Handle.run();
                ros::spinOnce();
                loop_rate.sleep();
        }
}