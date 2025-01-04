
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include "PID_controller.h"
#include "Param_and_data.h"

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
    Static_param::get_param(nh, "Drone/balance_throuttle", balance_throuttle);
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

Eigen::Vector3d PID_controller::lp3_filter(Lp3_filter lp3_filter, Eigen::Vector3d input) {
    lp3_filter.state1 = lp3_filter.state1 + lp3_filter.k * (input - lp3_filter.state1);
    lp3_filter.state2 = lp3_filter.state2 + lp3_filter.k * (lp3_filter.state1 - lp3_filter.state2);
    lp3_filter.state = lp3_filter.state + lp3_filter.k * (lp3_filter.state2 - lp3_filter.state);
    return lp3_filter.state;
}

Eigen::Vector3d PID_controller::ve2vb(Eigen::Vector3d input, double yaw){
    Eigen::Matrix3d R;
    R << cos(yaw), -sin(yaw), 0,
         sin(yaw), cos(yaw), 0,
         0, 0, 1;
    return R * input;
}

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

    desire_velocity_x = Kp_x * position_error[0] + temp_position_i_x;
    desire_velocity_y = Kp_y * position_error[1] + temp_position_i_y;
    desire_velocity_z = Kp_z * position_error[2] + temp_position_i_z;

}

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
    
}

void PID_controller::update_desire(Eigen::Vector3d desire_position){

}