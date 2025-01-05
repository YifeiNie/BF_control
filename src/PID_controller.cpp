
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include "PID_controller.h"
#include "Param_and_data.h"
#include "BFcontrol_node.h"

void PID_controller::init(ros::NodeHandle &nh){
    Static_param::get_param(nh, "PID/Kp_x", Kp_x);
    Static_param::get_param(nh, "PID/Kp_y", Kp_y);
    Static_param::get_param(nh, "PID/Kp_z", Kp_z);
    Static_param::get_param(nh, "PID/Ki_x", Ki_x);
    Static_param::get_param(nh, "PID/Ki_y", Ki_y);
    Static_param::get_param(nh, "PID/Ki_z", Ki_z);
    Static_param::get_param(nh, "PID/Kd_x", Kd_x);
    Static_param::get_param(nh, "PID/Kd_y", Kd_y);
    Static_param::get_param(nh, "PID/Kd_z", Kd_z);
    Static_param::get_param(nh, "PID/Kp_vx", Kp_vx);
    Static_param::get_param(nh, "PID/Kp_vy", Kp_vy);
    Static_param::get_param(nh, "PID/Kp_vz", Kp_vz);
    Static_param::get_param(nh, "PID/Ki_vx", Ki_vx);
    Static_param::get_param(nh, "PID/Ki_vy", Ki_vy);
    Static_param::get_param(nh, "PID/Ki_vz", Ki_vz);
    Static_param::get_param(nh, "PID/Kd_vx", Kd_vx);
    Static_param::get_param(nh, "PID/Kd_vy", Kd_vy);
    Static_param::get_param(nh, "PID/Kd_vz", Kd_vz);
    Static_param::get_param(nh, "PID/K_yaw", K_yaw);
    Static_param::get_param(nh, "PID/position_x_i_ub", position_x_i_ub);
    Static_param::get_param(nh, "PID/position_x_i_lb", position_x_i_lb);
    Static_param::get_param(nh, "PID/position_y_i_ub", position_y_i_ub);
    Static_param::get_param(nh, "PID/position_y_i_lb", position_y_i_lb);
    Static_param::get_param(nh, "PID/position_z_i_ub", position_z_i_ub);
    Static_param::get_param(nh, "PID/position_z_i_lb", position_z_i_lb);
    Static_param::get_param(nh, "PID/velocity_x_i_ub", velocity_x_i_ub);
    Static_param::get_param(nh, "PID/velocity_x_i_lb", velocity_x_i_lb);
    Static_param::get_param(nh, "PID/velocity_y_i_ub", velocity_y_i_ub);
    Static_param::get_param(nh, "PID/velocity_y_i_lb", velocity_y_i_lb);
    Static_param::get_param(nh, "PID/velocity_z_i_ub", velocity_z_i_ub);
    Static_param::get_param(nh, "PID/velocity_z_i_lb", velocity_z_i_lb);
    Static_param::get_param(nh, "Lowpass filter/lp3_k", lp3_k);
    Static_param::get_param(nh, "Drone/balance_thrust", balance_thrust);
    desire_position << 0, 0, 0;
    current_position << 0, 0, 0;
    current_velocity << 0, 0, 0;
    position_error_sum << 0, 0, 0;
    velocity_error_sum << 0, 0, 0;
}

void PID_controller::update_desire(Eigen::Vector3d desire_position) {
    this->desire_position = desire_position;
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
void PID_controller::outer_position_loop() {
    // lp3_filter<Eigen::Vector3d>(lp3_filter, desire_position);
    Eigen::Vector3d position_error = desire_position - current_position;
    position_error_sum += position_error / CTRL_FREQUENCY;
    
    double temp_position_i_x = Ki_x * position_error_sum[0];
    double temp_position_i_y = Ki_y * position_error_sum[1];
    double temp_position_i_z = Ki_z * position_error_sum[2];
    temp_position_i_x = limit(position_x_i_ub, position_x_i_lb, temp_position_i_x);
    temp_position_i_y = limit(position_y_i_ub, position_y_i_lb, temp_position_i_y);
    temp_position_i_z = limit(position_z_i_ub, position_z_i_lb, temp_position_i_z);

    desire_velocity[0] = Kp_x * position_error[0] + temp_position_i_x;
    desire_velocity[1] = Kp_y * position_error[1] + temp_position_i_y;
    desire_velocity[2] = Kp_z * position_error[2] + temp_position_i_z;
}

// 内环姿态控制
void PID_controller::outer_position_loop(){
    // desire_velocity = ve2vb(desire_velocity, odom_input.att[2]);
    Eigen::Vector3d velocity_error = desire_velocity - current_velocity;
    velocity_error_sum += velocity_error / CTRL_FREQUENCY;

    double temp_velocity_i_x = Ki_vx * velocity_error_sum[0];
    double temp_velocity_i_y = Ki_vy * velocity_error_sum[1];
    double temp_velocity_i_z = Ki_vz * velocity_error_sum[2];
    temp_velocity_i_x = limit(velocity_x_i_ub, velocity_x_i_lb, temp_velocity_i_x);
    temp_velocity_i_y = limit(velocity_y_i_ub, velocity_y_i_lb, temp_velocity_i_y);
    temp_velocity_i_z = limit(velocity_z_i_ub, velocity_z_i_lb, temp_velocity_i_z);

    // yaw的控制
    double yaw_error = desire_yaw - odom.quaternion2yaw(odom.q);

    // 设置飞控指令(body_rate的含义由att_cmd_msg.type_mask决定: 4是角度，1是角速度)
    att_cmd_msg.body_rate.x = (Kp_vx * velocity_error[0] + temp_velocity_i_x) * RAD2DEG;
    att_cmd_msg.body_rate.y = (Kp_vy * velocity_error[1] + temp_velocity_i_y) * RAD2DEG;
    att_cmd_msg.body_rate.z = K_yaw * yaw_error;
    att_cmd_msg.thrust = balance_thrust + Kp_vz * velocity_error[2] + temp_velocity_i_z;
    if(att_cmd_msg.thrust <= 0.01){
        att_cmd_msg.thrust = 0;
    }
        
}

// 设置期望位置
void PID_controller::update_desire_position(Eigen::Vector3d desire_position){
    this->desire_position = desire_position;
}