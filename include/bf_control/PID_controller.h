#pragma once 
#include <Eigen/Dense>
#include <topic_handler.h>

#define CTRL_FREQUENCY 150
#define RAD2DEG 57.29577951308232
#define DEG2RAD 0.017453292519943

struct Gain{
    double Kp_x, Kp_y, Kp_z;
    double Ki_x, Ki_y, Ki_z;
    double Kd_x, Kd_y, Kd_z;

    double Kp_vx, Kp_vy, Kp_vz;
    double Ki_vx, Ki_vy, Ki_vz;
    double Kd_vx, Kd_vy, Kd_vz;
    double K_yaw;
};

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
    Eigen::Vector3d desire_velocity ;           // 期望速度（储存外环的输出）
    Eigen::Vector3d current_velocity;
    double lp3_k;                               // 低通滤波器参数    
              
    aNYF
    
    Eigen::Vector3d position_error_sum;
    Eigen::Vector3d velocity_error_sum;
    double balance_thrust;                   // 平衡时的油门
    double position_x_i_ub, position_x_i_lb;     
    double position_y_i_ub, position_y_i_lb;    // 位置控制积分限幅
    double position_z_i_ub, position_z_i_lb;
    double velocity_x_i_ub, velocity_x_i_lb;
    double velocity_y_i_ub, velocity_y_i_lb;    // 速度控制积分限幅
    double velocity_z_i_ub, velocity_z_i_lb;
    double thrust_bound;                        // 油门限幅
    double x_bound, y_bound;                    // 姿态控制限幅

    Gain gain;                                  // PID参数结构体
    mavros_msgs::AttitudeTarget att_cmd_msg;    // 发到飞控的控制指令

    void init(ros::NodeHandle& nh);
    void outer_position_loop(Topic_handler& th);
    void inner_attitude_loop(Topic_handler& th);
    void update_desire(Eigen::Vector3d desire_position, double desire_yaw);
    double limit(double ub, double lb, double value);
    Eigen::Vector3d ve2vb(Eigen::Vector3d input, double yaw); // 全局坐标系到机体坐标系的转换
    Eigen::Vector3d lp3_filter(Lp3_filter lpf3, Eigen::Vector3d input);
};