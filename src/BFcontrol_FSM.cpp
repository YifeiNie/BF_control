#include "BFcontrol_FSM.h"
#include <std_msgs/UInt16MultiArray.h>

void BFcontrol_FSM::run(Topic_handler &th){
    ros::Time now_time = ros::Time::now();
    switch (state){
        case MANUAL_CTRL:
            pid.reset();
            // ROS_INFO("is_armed = %d", th.rc.is_armed);
            // ROS_INFO("is_offboard = %d", th.rc.is_offboard);
            // if(th.rc.is_offboard){
            if(th.rc.is_offboard && th.rc.is_armed && th.is_odom_received(now_time)){
                pid.reset();
                state = CMD_CTRL;
                pid.desire_position = th.odom.position;
                pid.desire_position[2] += 0.1;
                pid.desire_yaw = th.odom.get_current_yaw();
                ROS_INFO("x is %f", pid.desire_position[0]);
                ROS_INFO("y is %f", pid.desire_position[1]);
                ROS_INFO("z is %f", pid.desire_position[2]);
                // ROS_INFO("desire_yaw %f", pid.desire_yaw );
                ROS_INFO("Enter offboard mode!");
                // ROS_INFO("3333333333333333");
            }
            break;
        case CMD_CTRL:
            // if(!(th.rc.is_offboard)){
            if(!th.rc.is_offboard || !(th.is_odom_received(now_time))){
                state = MANUAL_CTRL;
                pid.reset();
                // ROS_INFO("2222222222222");
                ROS_INFO("Enter manual mode!");
            }
            else{
                pid.outer_position_loop(th);
                pid.inner_velocity_loop(th);
                // pid.inner_rate_loop(th);
                th.mav_cmd_publisher.publish(pid.att_cmd_msg);
                pid.desire_position[0] = 0.001 * (th.rc.roll-1505);
                pid.desire_position[1] = 0.001 * (th.rc.pitch-1505);
                pid.desire_position[2] = 0.0002 * (th.rc.thrust-505);

                double dyaw = M_PI + (th.rc.yaw - 1505.0) * 2 * M_PI / 1000.0;
                while (dyaw >= M_PI)  dyaw -= 2 * M_PI;
                while (dyaw < -M_PI)  dyaw += 2 * M_PI;
                pid.desire_yaw = -dyaw;
                // ROS_INFO("x is %f", pid.desire_position[0]);
                // ROS_INFO("y is %f", pid.desire_position[1]);
                // ROS_INFO("z is %f", pid.desire_position[2]);
                // ROS_INFO("yaw is %f", pid.desire_yaw);
                // ROS_INFO("x cmd is %f", pid.att_cmd_msg.body_rate.x);
                // ROS_INFO("y cmd is %f", pid.att_cmd_msg.body_rate.y);
                // ROS_INFO("thrust cmd is %f", pid.att_cmd_msg.thrust);
                // ROS_INFO("desire pos is %f", pid.desire_position[2]);
                // ROS_INFO("desire pos: x=%.3f y=%.3f z=%.3f", pid.desire_position[0], pid.desire_position[1], pid.desire_position[2]);
                // ROS_INFO("practical pos: x=%.3f y=%.3f z=%.3f", th.odom.position.x(), th.odom.position.y(), th.odom.position.z());
                // ROS_INFO_THROTTLE(0.5, "pos:  [%.2f %.2f %.2f] | vel: [%.2f %.2f %.2f]", th.odom.position.x(), th.odom.position.y(), th.odom.position.z(), th.odom.velocity.x(), th.odom.velocity.y(), th.odom.velocity.z());
                // ROS_INFO_THROTTLE(0.5, "cmd:  roll_rate=%.3f pitch_rate=%.3f yaw_rate=%.3f", pid.att_cmd_msg.body_rate.x, pid.att_cmd_msg.body_rate.y, pid.att_cmd_msg.body_rate.z);
                geometry_msgs::Vector3 v;

                v.x = th.odom.position.x();
                v.y = th.odom.position.y();
                v.z = th.odom.position.z();
                th.pos_pub.publish(v);
                // ROS_INFO("111111111111111111");
                v.x = th.odom.velocity.x();
                v.y = th.odom.velocity.y();
                v.z = th.odom.velocity.z();
                th.vel_pub.publish(v);

                v.x = pid.att_cmd_msg.body_rate.x;
                v.y = pid.att_cmd_msg.body_rate.y;
                v.z = pid.att_cmd_msg.body_rate.z;
                // v.z = pid.att_cmd_msg.thrust;
                th.cmd_pub.publish(v);

                v.x = th.imu.linear_acc.x();
                v.y = th.imu.linear_acc.y();
                v.z = th.imu.linear_acc.z();
                th.imu_linear_acc_pub.publish(v);

                v.x = th.imu.angle_vel.x();
                v.y = th.imu.angle_vel.y();
                v.z = th.imu.angle_vel.z();
                th.imu_angle_vel.publish(v);

                v.x = pid.velocity_error_sum[0];
                v.y = pid.velocity_error_sum[1];
                v.z = pid.velocity_error_sum[2];
                th.vel_error.publish(v);

                v.x = pid.position_error_sum[0];
                v.y = pid.position_error_sum[1];
                v.z = pid.position_error_sum[2];
                th.pos_error.publish(v);

                v.x = pid.yaw_error;
                v.y = pid.desire_yaw;
                v.z = pid.current_yaw_body;
                th.yaw_config.publish(v);

                // std_msgs::UInt16MultiArray rc_msg;
                // rc_msg.data.resize(4);  // roll, pitch, yaw, throttle
                // rc_msg.data[0] = pid.rc_roll;
                // rc_msg.data[1] = pid.rc_pitch;
                // rc_msg.data[2] = pid.rc_yaw;
                // rc_msg.data[3] = pid.rc_throttle;
                // th.rc_pub.publish(rc_msg);
            }
            break;
    }

}