#ifndef RES_HPP
#define RES_HPP

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <tr23_msg/ResStatus.h>

namespace ns_res {

class Res_Handle {

public:
        Res_Handle(ros::NodeHandle &nodeHandle);
        void run();
        void subscribeToTopics();
        void publishToTopics();

        void RES_Initialize();
        void RES_Set();
        void RES_Checkup();

private:
        tr23_msg::ResStatus RES_State;
        int PDO_00, PDO_01, PDO_02, PDO_37;

        ros::NodeHandle nodeHandle_;
        ros::Subscriber res_subscriber_;
        ros::Publisher res_publisher_;
        ros::Publisher state_publisher_;

        void resCallBack(const can_msgs::Frame &msg);
};

}



#endif