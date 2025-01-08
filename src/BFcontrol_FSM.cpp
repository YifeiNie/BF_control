#include "BFcontrol_FSM.h"

void BFcontrol_FSM::run(ros::NodeHandle &nh, Topic_handler &th){
    pid.desire_yaw = th.imu.get_current_yaw();
    pid.desire_position = th.odom.position;
    ros::Time now_time = ros::Time::now();
    switch (state){
        case MANUAL_CTRL:
            
            ROS_INFO("is_armed = %d", th.rc.is_armed);
            ROS_INFO("is_offboard = %d", th.rc.is_offboard);
            if(th.rc.is_offboard && th.rc.is_armed){
            // if(th.rc.is_offboard && th.rc.is_armed && th.is_odom_received(now_time)){
                state = CMD_CTRL;
                pid.update_desire(th.odom.position, th.imu.get_current_yaw());
                ROS_INFO("Enter offboard mode!");
            }
            break;
        case CMD_CTRL:
            if(!th.rc.is_offboard){
            // if(!th.rc.is_offboard || !(th.is_odom_received(now_time))){
                state = MANUAL_CTRL;
                ROS_INFO("Enter manual mode!");
            }
            else{
                pid.outer_position_loop(th);
                pid.inner_attitude_loop(th);
                th.mav_cmd_publisher.publish(pid.att_cmd_msg);
            }
            break;
    }

}