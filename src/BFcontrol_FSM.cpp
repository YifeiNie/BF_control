#include "BFcontrol_FSM.h"

void BFcontrol_FSM::run(Topic_handler &th){
    ros::Time now_time = ros::Time::now();
    switch (state){
        case MANUAL_CTRL:
            pid.reset();
            ROS_INFO("is_armed = %d", th.rc.is_armed);
            ROS_INFO("is_offboard = %d", th.rc.is_offboard);
            if(th.rc.is_offboard){
            // if(th.rc.is_offboard && th.rc.is_armed && th.is_odom_received(now_time)){
                pid.reset();
                state = CMD_CTRL;
                pid.desire_position = th.odom.position;
                pid.desire_position[2] += 0.4;
                pid.desire_yaw = th.imu.get_current_yaw();
                ROS_INFO("Enter offboard mode!");
            }
            break;
        case CMD_CTRL:
            if(!(th.rc.is_offboard)){
            // if(!th.rc.is_offboard || !(th.is_odom_received(now_time))){
                state = MANUAL_CTRL;
                pid.reset();
                ROS_INFO("Enter manual mode!");
            }
            else{
                pid.outer_position_loop(th);
                pid.inner_velocity_loop(th);
                th.mav_cmd_publisher.publish(pid.att_cmd_msg);
                ROS_INFO("x cmd is %f", pid.att_cmd_msg.body_rate.x);
                ROS_INFO("y cmd is %f", pid.att_cmd_msg.body_rate.y);
                ROS_INFO("thrust cmd is %f", pid.att_cmd_msg.thrust);
                ROS_INFO("desire pos is %f", pid.desire_position[2]);
            }
            break;
    }

}