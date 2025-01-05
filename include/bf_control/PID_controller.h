#pragma once 
#include <Eigen/Dense>
#include "Param_and_data.h"

#define CTRL_FREQUENCY 100

struct Lp3_filter {
    Eigen::Vector3d state;
    Eigen::Vector3d state1;
    Eigen::Vector3d state2;
    double k;
};

class PID_controller {

public:
    double desire_yaw;                          // 期望偏航角
    Eigen::Vector3d desire_position;            // 期望位置
    Eigen::Vector3d current_position;
    Eigen::Vector3d desire_velocity             // 期望速度（储存外环的输出）
    Eigen::Vector3d current_velocity;
    double lp3_k;                               // 低通滤波器参数    
              

    Eigen::Vector3d position_error_sum;
    Eigen::Vector3d velocity_error_sum;
    double balance_thrust;                   // 平衡时的油门
    double position_x_i_ub, position_x_i_lb;     
    double position_y_i_ub, position_y_i_lb;    // 位置控制积分限幅
    double position_z_i_ub, position_z_i_lb;
    double velocity_x_i_ub, velocity_x_i_lb;
    double velocity_y_i_ub, velocity_y_i_lb;    // 速度控制积分限幅
    double velocity_z_i_ub, velocity_z_i_lb;
    Lp3_filter lp3_filter;                      // 低通滤波器结构体
    Gain gain;                                  // PID参数结构体
    mavros_msgs::AttitudeTarget att_cmd_msg;    // 发到飞控的控制指令

    void init(ros::NodeHandle &nh);
    void outer_position_loop();
    void inner_attitude_loop();
    void update_desire(Eigen::Vector3d desire_position);
    Eigen::Vector3d ve2vb(double yaw); // 全局坐标系到机体坐标系的转换
    double limit(double ub, double lb, double value);
    Eigen::Vector3d lp3_filter(Lp3_filter lp3_filter, Eigen::Vector3d input);
    

};