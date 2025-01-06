#include "BFcontrol_FSM.h"

void BFcontrol_FSM::run(ros::NodeHandle &nh){
    pid.desire_yaw = th.imu.get_current_yaw();
    pid.desire_position = th.odom.position;
    ros::Time now_time = ros::Time::now();
    switch (state){
        case MANUAL_CTRL:
            if(th.rc.is_offboard && th.rc.is_armed && th.is_odom_received(now_time)){
                state = CMD_CTRL;
            }
            break;
        case CMD_CTRL:
            if(!th.rc.is_offboard || !th.is_odom_received(now_time)){
                state = MANUAL_CTRL;
            }
            else{
                Eigen::Vector3d pos(0, 0, 0.3);
                pid.update_desire(pos, th.imu.get_current_yaw());
                pid.outer_position_loop(pid.desire_position, &th);
                pid.inner_attitude_loop(&th);
                th.mav_cmd_publisher.publish(pid.att_cmd_msg);
            }
            break;
    }

}