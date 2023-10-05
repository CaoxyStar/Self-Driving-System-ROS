#include<ros/ros.h>
#include"can_msgs/Frame.h"
#include"../include/can_signal_sub/json_tool.h"
#include"../include/can_signal_sub/qnode.hpp"
using namespace std;

can_msgs::Frame frameBuffer;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_signal_subscriber");
    ros::NodeHandle n;
    can_signal_sub_Handle can_signal_handle(n);

    //读取全部信号
    read_json(can_signal_handle.All_signal_list);

    // Read line_fsc signals
    vector<string>LineFSC_name_list;
    string LineFSC_file = "/home/xinyh/TJURacing/TJURacing_Autonomous_System/ros/src/drivers/can_comms/can_signal_sub/config/line_fsc_signals.txt";
    read_signal_name_list(LineFSC_file, LineFSC_name_list);
    signal_select(can_signal_handle.All_signal_list, can_signal_handle.LineFSC_signal_list, LineFSC_name_list);

    //运行ros
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        //can_signal_handle.run_();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
