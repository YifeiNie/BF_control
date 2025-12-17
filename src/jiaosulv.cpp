
// #include <mavros_msgs/AttitudeTarget.h>
// #include <ros/ros.h>
// #include "PID_controller.h"
// #include "topic_handler.h"

// void PID_controller::init(ros::NodeHandle &nh){
//     // 如果是全局参数需要在最前面夹斜杠，但是我这是四有参数，所以不加
//     nh.getParam("PID/Kp_x", gain.Kp_x);
//     nh.getParam("PID/Kp_y", gain.Kp_y);
//     nh.getParam("PID/Kp_z", gain.Kp_z);
//     nh.getParam("PID/Ki_x", gain.Ki_x);
//     nh.getParam("PID/Ki_y", gain.Ki_y);
//     nh.getParam("PID/Ki_z", gain.Ki_z);
//     nh.getParam("PID/Kd_x", gain.Kd_x);
//     nh.getParam("PID/Kd_y", gain.Kd_y);
//     nh.getParam("PID/Kd_z", gain.Kd_z);
//     nh.getParam("PID/Kp_vx", gain.Kp_vx);
//     nh.getParam("PID/Kp_vy", gain.Kp_vy);
//     nh.getParam("PID/Kp_vz", gain.Kp_vz);
//     nh.getParam("PID/Ki_vx", gain.Ki_vx);
//     nh.getParam("PID/Ki_vy", gain.Ki_vy);
//     nh.getParam("PID/Ki_vz", gain.Ki_vz);
//     nh.getParam("PID/Kd_vx", gain.Kd_vx);
//     nh.getParam("PID/Kd_vy", gain.Kd_vy);
//     nh.getParam("PID/Kd_vz", gain.Kd_vz);
//     nh.getParam("PID/K_yaw", gain.K_yaw);

//     nh.getParam("PID/Ki_vy", gain.Kp_att_roll);
//     nh.getParam("PID/Ki_vz", gain.Kp_att_pitch);
//     nh.getParam("PID/Kd_vx", gain.Kp_att_yaw);
//     nh.getParam("PID/Kd_vy", gain.Kp_rate_roll);
//     nh.getParam("PID/Kd_vz", gain.Kp_rate_pitch);
//     nh.getParam("PID/Kd_vz", gain.Kp_rate_yaw);    

//     nh.getParam("PID/position_x_i_bound", position_x_i_bound);
//     nh.getParam("PID/position_y_i_bound", position_y_i_bound);
//     nh.getParam("PID/position_z_i_bound", position_z_i_bound);
//     nh.getParam("PID/velocity_x_i_bound", velocity_x_i_bound);
//     nh.getParam("PID/velocity_y_i_bound", velocity_y_i_bound);
//     nh.getParam("PID/velocity_z_i_bound", velocity_z_i_bound);
//     nh.getParam("PID/x_bound", x_bound);
//     nh.getParam("PID/y_bound", y_bound);
//     nh.getParam("PID/thrust_bound", thrust_bound);
//     nh.getParam("Lowpass_filter/lp3_k", lp3_k);
//     nh.getParam("Drone/balance_thrust", balance_thrust);

//     nh.getParam("PID_single/Kpx_single", gain.Kp_x_single);
//     nh.getParam("PID_single/Kpy_single", gain.Kp_y_single);
//     nh.getParam("PID_single/Kpz_single", gain.Kp_z_single);
//     nh.getParam("PID_single/Kvx_single", gain.Kv_x_single);
//     nh.getParam("PID_single/Kvy_single", gain.Kv_y_single);
//     nh.getParam("PID_single/Kvz_single", gain.Kv_z_single);
//     // nh.getParam("Drone/g", balance_thrust);
//     gain.Kp_single << gain.Kp_x_single, gain.Kp_y_single, gain.Kp_z_single;
//     gain.Kv_single << gain.Kv_x_single, gain.Kv_y_single, gain.Kv_z_single;
//     desire_position << 0, 0, 0;
//     current_position << 0, 0, 0;
//     current_velocity << 0, 0, 0;
//     position_error_sum << 0, 0, 0;
//     velocity_error_sum << 0, 0, 0;
//     g_vector << 0, 0, G;
// }

// void PID_controller::reset(){
//     this->velocity_error_sum.setZero();
//     this->position_error_sum.setZero();
//     this->desire_position.setZero();
//     this->desire_velocity.setZero();
//     this->desire_accelerate.setZero();
//     this->desire_jerk.setZero();
//     this->desire_yaw = 0;

//     att_cmd_msg.body_rate.x = 0;
//     att_cmd_msg.body_rate.y = 0;
//     att_cmd_msg.body_rate.z = 0;
//     att_cmd_msg.thrust = 0;
// }

// double PID_controller::limit(double ub, double lb, double value) {
//     if(value > ub)
//         value = ub;
//     if(value < lb)
//         value = lb;
//     return value;
// }

// // 3阶低通滤波器
// Eigen::Vector3d PID_controller::lp3_filter(Lp3_filter lp3_filter, Eigen::Vector3d input) {
//     lp3_filter.state1 = lp3_filter.state1 + lp3_filter.k * (input - lp3_filter.state1);
//     lp3_filter.state2 = lp3_filter.state2 + lp3_filter.k * (lp3_filter.state1 - lp3_filter.state2);
//     lp3_filter.state = lp3_filter.state + lp3_filter.k * (lp3_filter.state2 - lp3_filter.state);
//     return lp3_filter.state;
// }

// // 世界坐标系转机体坐标系
// Eigen::Vector3d PID_controller::
// ve2vb(Eigen::Vector3d input, double yaw){
//     Eigen::Matrix3d R;
//     R << cos(yaw), sin(yaw), 0,
//          -sin(yaw), cos(yaw), 0,
//          0, 0, 1;
//     return R * input;
// }

// // 外环位置控制
// void PID_controller::outer_position_loop(Topic_handler& th) {
//     // 位置误差 = 期望位置 - 当前位置
//     Eigen::Vector3d position_error = desire_position - th.odom.position;
//     // ROS_INFO("x cmd is %f", position_error[0]);
//     // ROS_INFO("y cmd is %f", position_error[1]);
//     // ROS_INFO("thrust cmd is %f", position_error[2]);
//     position_error_sum += position_error / CTRL_FREQUENCY;
    
//     double temp_position_i_x = gain.Ki_x * position_error_sum[0];
//     double temp_position_i_y = gain.Ki_y * position_error_sum[1];
//     double temp_position_i_z = gain.Ki_z * position_error_sum[2];
//     temp_position_i_x = limit(position_x_i_bound, -position_x_i_bound, temp_position_i_x);
//     temp_position_i_y = limit(position_y_i_bound, -position_y_i_bound, temp_position_i_y);
//     temp_position_i_z = limit(position_z_i_bound, -position_z_i_bound, temp_position_i_z);

//     desire_velocity[0] = gain.Kp_x * position_error[0] + temp_position_i_x;
//     desire_velocity[1] = gain.Kp_y * position_error[1] + temp_position_i_y;
//     desire_velocity[2] = gain.Kp_z * position_error[2] + temp_position_i_z;

//     double current_yaw_body = th.odom.get_current_yaw();
//     desire_velocity = ve2vb(desire_velocity, current_yaw_body);
// }

// Eigen::Vector3d PID_controller::velocity_loop_acc_cmd(Topic_handler& th) {

//     double current_yaw_body = th.odom.get_current_yaw();

//     // 机体系速度误差
//     Eigen::Vector3d desire_velocity = ve2vb(desire_velocity, current_yaw_body);
//     Eigen::Vector3d velocity_error = desire_velocity - th.odom.velocity;

//     // 积分项
//     velocity_error_sum += velocity_error/ CTRL_FREQUENCY;

//     Eigen::Vector3d vel_i;
//     vel_i[0] = limit(velocity_x_i_bound, -velocity_x_i_bound, gain.Ki_vx * velocity_error_sum[0]);
//     vel_i[1] = limit(velocity_y_i_bound, -velocity_y_i_bound, gain.Ki_vy * velocity_error_sum[1]);
//     vel_i[2] = limit(velocity_z_i_bound, -velocity_z_i_bound, gain.Ki_vz * velocity_error_sum[2]);

//     // 输出期望加速度（速度→加速度）
//     Eigen::Vector3d acc_des;
//     acc_des[0] = gain.Kp_vx * velocity_error[0] + vel_i[0];
//     acc_des[1] = gain.Kp_vy * velocity_error[1] + vel_i[1];
//     acc_des[2] = gain.Kp_vz * velocity_error[2] + vel_i[2];

//     return acc_des;
// }

// Eigen::Vector3d PID_controller::acc_to_attitude(Eigen::Vector3d acc_des, double yaw) {

//     double g = 9.80665;

//     double ax = acc_des[0];
//     double ay = acc_des[1];
//     double az = acc_des[2] + g;  // 加上重力补偿

//     // 标准转换：期望加速度 → roll/pitch
//     double theta_des =  ( ax * cos(yaw) + ay * sin(yaw) ) / g;  // pitch
//     double phi_des   =  ( ax * sin(yaw) - ay * cos(yaw) ) / g;  // roll

//     // 限幅
//     phi_des   = limit(0.5, -0.5, phi_des);
//     theta_des = limit(0.5, -0.5, theta_des);

//     return Eigen::Vector3d(phi_des, theta_des, desire_yaw);
// }

// Eigen::Vector3d PID_controller::attitude_loop_rate_cmd(Topic_handler& th, Eigen::Vector3d att_des) {

//     double yaw = th.odom.get_current_yaw();

//     // 当前姿态
//     Eigen::Vector3d att;

//     Eigen::Vector3d euler;
//     Eigen::Matrix3d R = th.odom.q.toRotationMatrix();
//     euler = R.eulerAngles(0, 1, 2);  // roll pitch yaw
//     att[0] = euler[0]; // roll
//     att[1] = euler[1]; // pitch
//     att[2] = euler[2]; // yaw

//     // 姿态误差
//     Eigen::Vector3d att_err = att_des - att;

//     // P控制（足够）
//     Eigen::Vector3d rate_des;
//     rate_des[0] = gain.Kp_att_roll  * att_err[0];
//     rate_des[1] = gain.Kp_att_pitch * att_err[1];
//     rate_des[2] = gain.Kp_att_yaw   * att_err[2];

//     return rate_des;
// }

// void PID_controller::inner_rate_loop(Topic_handler& th) {

//     double yaw = th.odom.get_current_yaw();

//     // 1. 速度环 → 期望加速度
//     Eigen::Vector3d acc_des = velocity_loop_acc_cmd(th);

//     // 2. 期望加速度 → 期望姿态
//     Eigen::Vector3d att_des = acc_to_attitude(acc_des, yaw);

//     // 3. 姿态环 → 期望角速度
//     Eigen::Vector3d rate_des = attitude_loop_rate_cmd(th, att_des);

//     // 4. 角速度误差
//     Eigen::Vector3d rate_err = rate_des - th.imu.angle_vel;

//     // 5. 角速度 P 控制（可加 I）
//     Eigen::Vector3d rate_cmd;
//     rate_cmd[0] = gain.Kp_rate_roll  * rate_err[0];
//     rate_cmd[1] = gain.Kp_rate_pitch * rate_err[1];
//     rate_cmd[2] = gain.Kp_rate_yaw   * rate_err[2];

//     // Thrust 单独处理
//     double thrust_cmd = acc_des[2] + balance_thrust;

//     // 6. 发布最终控制指令
//     att_cmd_msg.type_mask = 1;   // 按角速度控制
//     att_cmd_msg.body_rate.x = rate_cmd[0] * RAD2DEG;
//     att_cmd_msg.body_rate.y = rate_cmd[1] * RAD2DEG;
//     att_cmd_msg.body_rate.z = rate_cmd[2] * RAD2DEG;
//     att_cmd_msg.thrust = limit(thrust_bound, -thrust_bound, thrust_cmd);
// }


// // // 内环姿态控制
// // void PID_controller::inner_velocity_loop(Topic_handler& th){

// //     // 获取当前yaw角用于将全局坐标系的线速度转换为机体坐标系的线速度
// //     double current_yaw_body = th.odom.get_current_yaw();
// //     desire_velocity = ve2vb(desire_velocity, current_yaw_body);
// //     // 速度误差 = 期望速度 - 当前速度
// //     Eigen::Vector3d velocity_error = desire_velocity - th.odom.velocity ;

// //     // yaw的控制
// //     double yaw_error = desire_yaw - current_yaw_body;
// //     velocity_error_sum += velocity_error / CTRL_FREQUENCY;

// //     double temp_velocity_i_x = gain.Ki_vx * velocity_error_sum[0];
// //     double temp_velocity_i_y = gain.Ki_vy * velocity_error_sum[1];
// //     double temp_velocity_i_z = gain.Ki_vz * velocity_error_sum[2];
// //     temp_velocity_i_x = limit(velocity_x_i_bound, -velocity_x_i_bound, temp_velocity_i_x);
// //     temp_velocity_i_y = limit(velocity_y_i_bound, -velocity_y_i_bound, temp_velocity_i_y);
// //     temp_velocity_i_z = limit(velocity_z_i_bound, -velocity_z_i_bound, temp_velocity_i_z);

// //     double temp_x_out = (gain.Kp_vx * velocity_error[0] + temp_velocity_i_x);
// //     double temp_y_out = (gain.Kp_vy * velocity_error[1] + temp_velocity_i_y);
// //     double temp_thrust_out = balance_thrust + gain.Kp_vz * velocity_error[2] + temp_velocity_i_z;
// //     double temp_yaw_out = gain.K_yaw * yaw_error;

// //     temp_x_out = limit(x_bound, -x_bound, temp_x_out);
// //     temp_y_out = limit(y_bound, -y_bound, temp_y_out);
// //     temp_thrust_out = limit(thrust_bound, -thrust_bound, temp_thrust_out);  

// //     // 设置飞控指令(body_rate的含义由att_cmd_msg.type_mask决定: 4是角度，1是角速度)
// //     att_cmd_msg.type_mask = 4;
// //     att_cmd_msg.body_rate.x = -temp_y_out * RAD2DEG;
// //     att_cmd_msg.body_rate.y = temp_x_out * RAD2DEG;
// //     att_cmd_msg.body_rate.z = temp_yaw_out;
// //     att_cmd_msg.thrust = temp_thrust_out;
// //     if(abs(att_cmd_msg.thrust) <= 0.01){
// //         att_cmd_msg.thrust = 0;
// //     }    
// // }

// void PID_controller::single_accelerate_loop(Topic_handler& th){
//     // double current_yaw = th.imu.get_current_yaw();
//     // double sin = std::sin(current_yaw);
//     // double cos = std::cos(current_yaw);
//     // Eigen::Vector3d  temp_acc_output = desire_accelerate + 
//     //                                gain.Kv_single.asDiagonal() * (desire_velocity - th.odom.velocity) + 
//     //                                gain.Kp_single.asDiagonal() * (desire_position - th.odom.position);
//     // temp_acc_output += g_vector;
//     // double roll_out = (temp_acc_output(0)*sin - temp_acc_output(1)*cos )/G;
//     // double pitch_out = (temp_acc_output(0)*cos + temp_acc_output(1)*sin )/G;
//     // double yaw_out = this->desire_yaw;
//     // double thrust_out = get_thrust_out();
    
//     // att_cmd_msg.type_mask = 4;
//     // att_cmd_msg.body_rate.x = roll_out * RAD2DEG;
//     // att_cmd_msg.body_rate.y = pitch_out * RAD2DEG;
//     // att_cmd_msg.body_rate.z = yaw_out;
//     // att_cmd_msg.thrust = thrust_out;
//     // if(abs(att_cmd_msg.thrust) <= 0.01){
//     //     att_cmd_msg.thrust = 0;
//     // }    
// }

// // bool PID_controller::estimateThrustModel(const Eigen::Vector3d &est_a) {
// //     ros::Time t_now = ros::Time::now();
// //     while (timed_thrust.size() >= 1) {
// //         // Choose data before 35~45ms ago
// //         std::pair<ros::Time, double> t_t = timed_thrust.front();
// //         double time_passed = (t_now - t_t.first).toSec();
// //         if (time_passed > 0.045) {  // 45ms
// //             // printf("continue, time_passed=%f\n", time_passed);
// //             timed_thrust.pop();
// //             continue;
// //         }
// //         if (time_passed < 0.035) {  // 35ms
// //             // printf("skip, time_passed=%f\n", time_passed);
// //             return false;
// //         }

// //         /***********************************************************/
// //         /* Recursive least squares algorithm with vanishing memory */
// //         /***********************************************************/
// //         double thr = t_t.second;
// //         timed_thrust.pop();
        
// //         /***********************************/
// //         /* Model: est_a(2) = thr1acc_ * thr */
// //         /***********************************/
// //         double gamma = 1 / (rho + thr * P * thr);
// //         double K = gamma * P * thr;
// //         thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
// //         P = (1 - K * thr) * P / rho;
// //         //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
// //         //fflush(stdout);

// //         // debug_msg_.thr2acc = thr2acc_;
// //         return true;
// //     }
// //     return false;
// // }
 