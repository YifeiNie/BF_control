#include <ros/ros.h>
#include "BFcontrol_FSM.h"
#include "topic_handler.h"
#include <PID_controller.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh("~");
    BFcontrol_FSM fsm;
    fsm.pid.init(nh);
    fsm.th.topic_handler_init(nh);
    ros::Rate rate(10);
    while (ros::ok()){
        ros::spinOnce();
        fsm.run(nh);
        rate.sleep();
    }
  return 0;
}