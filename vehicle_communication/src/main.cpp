#include "../include/vehicle_communication/vehicle_communication.hpp"

using namespace ns_VehicleCommunication;

int main(int argc, char** argv) {
        ros::init(argc,  argv, "vehicle_communication_node") ;
        ros::NodeHandle nodeHandle;
        VehicleCommunication myCommunication_Handle(nodeHandle);
        ros::Rate loop_rate(10);
        while(ros::ok()) {
                myCommunication_Handle.run();
                ros::spinOnce();
                loop_rate.sleep();
        }
}