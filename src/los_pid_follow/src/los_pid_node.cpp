#include "los_pid_follow/los_pid.h"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "los_pid_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    auto_recovery::LOS_PID los_pid(nh, nh_p);

    ros::spin();
    return 0;
}