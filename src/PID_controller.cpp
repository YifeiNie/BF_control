
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include "PID_controller.h"
#include "topic_handler.h"

void PID_controller::init(ros::NodeHandle &nh){
    nh.getParam("PID/Kp_x", gain.Kp_x);
    nh.getParam("PID/Kp_y", gain.Kp_y);
    nh.getParam("PID/Kp_z", gain.Kp_z);
    nh.getParam("PID/Ki_x", gain.Ki_x);
    nh.getParam("PID/Ki_y", gain.Ki_y);
    nh.getParam("PID/Ki_z", gain.Ki_z);
    nh.getParam("PID/Kd_x", gain.Kd_x);
    nh.getParam("PID/Kd_y", gain.Kd_y);
    nh.getParam("PID/Kd_z", gain.Kd_z);
    nh.getParam("PID/Kp_vx", gain.Kp_vx);
    nh.getParam("PID/Kp_vy", gain.Kp_vy);
    nh.getParam("PID/Kp_vz", gain.Kp_vz);
    nh.getParam("PID/Ki_vx", gain.Ki_vx);
    nh.getParam("PID/Ki_vy", gain.Ki_vy);
    nh.getParam("PID/Ki_vz", gain.Ki_vz);
    nh.getParam("PID/Kd_vx", gain.Kd_vx);
    nh.getParam("PID/Kd_vy", gain.Kd_vy);
    nh.getParam("PID/Kd_vz", gain.Kd_vz);
    nh.getParam("PID/K_yaw", gain.K_yaw);
    nh.getParam("PID/position_x_i_ub", position_x_i_ub);
    nh.getParam("PID/position_x_i_lb", position_x_i_lb);
    nh.getParam("PID/position_y_i_ub", position_y_i_ub);
    nh.getParam("PID/position_y_i_lb", position_y_i_lb);
    nh.getParam("PID/position_z_i_ub", position_z_i_ub);
    nh.getParam("PID/position_z_i_lb", position_z_i_lb);
    nh.getParam("PID/velocity_x_i_ub", velocity_x_i_ub);
    nh.getParam("PID/velocity_x_i_lb", velocity_x_i_lb);
    nh.getParam("PID/velocity_y_i_ub", velocity_y_i_ub);
    nh.getParam("PID/velocity_y_i_lb", velocity_y_i_lb);
    nh.getParam("PID/velocity_z_i_ub", velocity_z_i_ub);
    nh.getParam("PID/velocity_z_i_lb", velocity_z_i_lb);
    nh.getParam("Lowpass filter/lp3_k", lp3_k);
    nh.getParam("Drone/balance_thrust", balance_thrust);
    desire_position << 0, 0, 0;
    current_position << 0, 0, 0;
    current_velocity << 0, 0, 0;
    position_error_sum << 0, 0, 0;
    velocity_error_sum << 0, 0, 0;
}

void PID_controller::update_desire(Eigen::Vector3d desire_position, double desire_yaw) {
    this->desire_position = desire_position;
    this->desire_yaw = desire_yaw;
}

double PID_controller::limit(double ub, double lb, double value) {
    if(value > ub)
        value = ub;
    if(value < lb)
        value = lb;
    return value;
}

// 3阶低通滤波器
Eigen::Vector3d PID_controller::lp3_filter(Lp3_filter lp3_filter, Eigen::Vector3d input) {
    lp3_filter.state1 = lp3_filter.state1 + lp3_filter.k * (input - lp3_filter.state1);
    lp3_filter.state2 = lp3_filter.state2 + lp3_filter.k * (lp3_filter.state1 - lp3_filter.state2);
    lp3_filter.state = lp3_filter.state + lp3_filter.k * (lp3_filter.state2 - lp3_filter.state);
    return lp3_filter.state;
}

// 世界坐标系转机体坐标系
Eigen::Vector3d PID_controller::ve2vb(Eigen::Vector3d input, double yaw){
    Eigen::Matrix3d R;
    R << cos(yaw), -sin(yaw), 0,
         sin(yaw), cos(yaw), 0,
         0, 0, 1;
    return R * input;
}

// 外环位置控制
void PID_controller::outer_position_loop(Eigen::Vector3d desire_position, Topic_handler* th) {
    // 位置误差 = 期望位置 - 当前位置
    Eigen::Vector3d position_error = desire_position - th->odom.position;
    position_error_sum += position_error / CTRL_FREQUENCY;
    
    double temp_position_i_x = gain.Ki_x * position_error_sum[0];
    double temp_position_i_y = gain.Ki_y * position_error_sum[1];
    double temp_position_i_z = gain.Ki_z * position_error_sum[2];
    temp_position_i_x = limit(position_x_i_ub, position_x_i_lb, temp_position_i_x);
    temp_position_i_y = limit(position_y_i_ub, position_y_i_lb, temp_position_i_y);
    temp_position_i_z = limit(position_z_i_ub, position_z_i_lb, temp_position_i_z);

    desire_velocity[0] = gain.Kp_x * position_error[0] + temp_position_i_x;
    desire_velocity[1] = gain.Kp_y * position_error[1] + temp_position_i_y;
    desire_velocity[2] = gain.Kp_z * position_error[2] + temp_position_i_z;
}

// 内环姿态控制
void PID_controller::inner_attitude_loop(Topic_handler* th){

    // 获取当前yaw角用于将全局坐标系的线速度转换为机体坐标系的线速度
    double current_yaw_body = th->imu.get_current_yaw();
    desire_velocity = ve2vb(desire_velocity, current_yaw_body);
    // 速度误差 = 期望速度 - 当前速度
    Eigen::Vector3d velocity_error = desire_velocity - th->imu.linear_acc;
    velocity_error_sum += velocity_error / CTRL_FREQUENCY;

    double temp_velocity_i_x = gain.Ki_vx * velocity_error_sum[0];
    double temp_velocity_i_y = gain.Ki_vy * velocity_error_sum[1];
    double temp_velocity_i_z = gain.Ki_vz * velocity_error_sum[2];
    temp_velocity_i_x = limit(velocity_x_i_ub, velocity_x_i_lb, temp_velocity_i_x);
    temp_velocity_i_y = limit(velocity_y_i_ub, velocity_y_i_lb, temp_velocity_i_y);
    temp_velocity_i_z = limit(velocity_z_i_ub, velocity_z_i_lb, temp_velocity_i_z);

    // yaw的控制
    double yaw_error = desire_yaw - current_yaw_body;

    // 设置飞控指令(body_rate的含义由att_cmd_msg.type_mask决定: 4是角度，1是角速度)
    att_cmd_msg.type_mask = 4;
    att_cmd_msg.body_rate.x = (gain.Kp_vx * velocity_error[0] + temp_velocity_i_x) * RAD2DEG;
    att_cmd_msg.body_rate.y = (gain.Kp_vy * velocity_error[1] + temp_velocity_i_y) * RAD2DEG;
    att_cmd_msg.body_rate.z = gain.K_yaw * yaw_error;
    att_cmd_msg.thrust = balance_thrust + gain.Kp_vz * velocity_error[2] + temp_velocity_i_z;
    if(att_cmd_msg.thrust <= 0.01){
        att_cmd_msg.thrust = 0;
    }
        
}
