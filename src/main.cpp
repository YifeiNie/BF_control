#include <ros/ros.h>
#include "BFcontrol_FSM.h"
#include "topic_handler.h"
#include <PID_controller.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "offboard_node");

    ros::NodeHandle nh("~");
    BFcontrol_FSM fsm;      // 用于执行程序主体
    Topic_handler th;       // 实时更新并存储话题数据
    th.topic_handler_init(nh, th);
    fsm.pid.init(nh);
    ros::Rate rate(10);
    while (ros::ok()){
        ros::spinOnce();
        fsm.run(nh, th);
        rate.sleep();
    }
    return 0;
}