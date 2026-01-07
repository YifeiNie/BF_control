#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>

#include "topic_handler.h"
  
bool Topic_handler::is_rc_received(const ros::Time &now_time){
	return (now_time - rc.rcv_stamp).toSec() < 0.5;   // 设置0.5秒为超时
}

bool Topic_handler::is_odom_received(const ros::Time &now_time){
    return (now_time - odom.rcv_stamp).toSec() < 0.5;
}

bool Topic_handler::is_imu_received(const ros::Time &now_time){
    return (now_time - imu.rcv_stamp).toSec() < 0.5;
}

// void Topic_handler::update_yaw_offset() {
//     if (!odom.yaw_offset_initialized && this->is_imu_received(ros::Time::now())) {
        
//         // 将 T265 yaw 转换到机体坐标系 NED
//     double odom_yaw = odom.get_current_yaw();

//     // 将 IMU yaw 转到同一参考方向
//     double imu_yaw = imu.get_current_yaw(); // 如果需要，加符号调整

//     // 偏置 = T265 初始 yaw - IMU 初始 yaw
//     odom.yaw_offset = odom_yaw - imu_yaw;

//     // 保证在 [-pi, pi] 范围
//     if (odom.yaw_offset > M_PI) odom.yaw_offset -= 2*M_PI;
//     if (odom.yaw_offset < -M_PI) odom.yaw_offset += 2*M_PI;

//     odom.yaw_offset_initialized = true;
//     ROS_INFO_STREAM("Initialized yaw offset: " << odom.yaw_offset * 180.0 / M_PI << " deg");
//     }
// }



void RC::feed(mavros_msgs::RCInConstPtr msg){
    roll = msg->channels[0];
    pitch = msg->channels[1];
    yaw = msg->channels[2];
    thrust = msg->channels[3];
    if (msg->channels[7] > 1700){
        is_armed = 1;
    }else{
        is_armed = 0;
    }

    if (msg->channels[6] > 1700){
        is_offboard = 1;
    }else{
        is_offboard = 0;
    }
    rcv_stamp = ros::Time::now();
    // ROS_INFO("channel 7 = %d",msg->channels[7] );
    // ROS_INFO("is_offboard = %d", is_offboard);
    // ROS_INFO("roll = %d", roll);
}

//传统坐标系
void Odom::feed(nav_msgs::OdometryConstPtr msg){

    Eigen::Matrix3d R_offset;
    R_offset = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());

    position(0) = msg->pose.pose.position.x;
    position(1) = msg->pose.pose.position.y;
    position(2) = msg->pose.pose.position.z;

        // 第一次接收 T265 数据时计算偏移量
    if (!offset_initialized) {
        t265_offset = desired_origin - position;
        offset_initialized = true;
        ROS_INFO_STREAM("T265 offset initialized: " << t265_offset.transpose());
    }
    position = R_offset * position;

    // // 加上偏移量
    // position = position + t265_offset;


    velocity(0) = msg->twist.twist.linear.x;
    velocity(1) = msg->twist.twist.linear.y;
    velocity(2) = msg->twist.twist.linear.z;
    velocity = R_offset * velocity;
    
    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    Eigen::Quaterniond q_offset(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
    q = q_offset * q; 
    rcv_stamp = ros::Time::now();
    
    // ROS_INFO("Get odom data!!!");
}

//T265坐标系转换到机体坐标系下
// void Odom::feed(nav_msgs::OdometryConstPtr msg){

//     // ENU → NED (位置)
//     position(0) =  msg->pose.pose.position.y;       // North  = ENU.y
//     position(1) =  msg->pose.pose.position.x;       // East   = ENU.x
//     position(2) = -msg->pose.pose.position.z;       // Down   = -Up

    
//     // 第一次接收 T265 数据时计算偏移量
//     if (!offset_initialized) {
//         t265_offset = desired_origin - position;
//         offset_initialized = true;
//         ROS_INFO_STREAM("T265 offset initialized: " << t265_offset.transpose());
//     }

//     // 加上偏移量
//     position = position + t265_offset;


//     // ENU → NED (速度)
//     velocity(0) =  msg->twist.twist.linear.y;
//     velocity(1) =  msg->twist.twist.linear.x;
//     velocity(2) = -msg->twist.twist.linear.z;

//     // quaternion ENU → NED
//     // conversion: q_ned = R * q_enu where R = [0 1 0; 1 0 0; 0 0 -1]
//     Eigen::Quaterniond q_enu(
//         msg->pose.pose.orientation.w,
//         msg->pose.pose.orientation.x,
//         msg->pose.pose.orientation.y,
//         msg->pose.pose.orientation.z
//     );

//     Eigen::Matrix3d R_enu_to_ned;
//     R_enu_to_ned << 0, 1, 0,
//                     1, 0, 0,
//                     0, 0, -1;

//     Eigen::Matrix3d Rn = R_enu_to_ned * q_enu.toRotationMatrix();
//     q = Eigen::Quaterniond(Rn);

//     rcv_stamp = ros::Time::now();
// }

//传统yaw获取方式
double Odom::get_current_yaw() const {
    double yaw = M_PI/2 + atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    while (yaw > M_PI) yaw -= 2*M_PI;
    while (yaw < -M_PI) yaw += 2*M_PI;
    return yaw;
}

// //T265yaw获取方式
// double Odom::get_current_yaw() const {
//     // 从四元数提取yaw
//     // NED下的四元数 q = [w, x, y, z]
//     Eigen::Matrix3d R = q.toRotationMatrix();
//     double yaw = std::atan2(R(1,0), R(0,0));  // NED坐标系
//     if (yaw_offset_initialized)
//         yaw -= yaw_offset;  
//     return yaw;
// }

void Imu::feed(sensor_msgs::ImuConstPtr msg){
    linear_acc(0) = msg->linear_acceleration.x;
    linear_acc(1) = msg->linear_acceleration.y;
    linear_acc(2) = msg->linear_acceleration.z;
    q.x() = msg->orientation.x;
    q.y() = msg->orientation.y;
    q.z() = msg->orientation.z;
    q.w() = msg->orientation.w;
    angle_vel(0) = msg->angular_velocity.x;
    angle_vel(1) = msg->angular_velocity.y;
    angle_vel(2) = msg->angular_velocity.z;
    rcv_stamp = ros::Time::now();
    // ROS_INFO("Get imu data!!!");
}

//没相机时获得yaw角不需要考虑偏差，不对yaw做控制
double Imu::get_current_yaw(){
    double yaw = atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    
    return yaw;
}

// //用相机矫正yaw角
// double Imu::get_current_yaw(double t265_yaw, bool use_offset){
//     double yaw = atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());

//     if(use_offset && imu_yaw_offset_initialized){
//         yaw -= imu_yaw_offset;
//     }

//     return yaw;
// }



//检测角度
void Topic_handler::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    // 先让 Odom 做 NED 转换
    this->odom.feed(msg);

    // 再发布 NED Odom
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "ned";

    odom_msg.pose.pose.position.x = odom.position(0);
    odom_msg.pose.pose.position.y = odom.position(1);
    odom_msg.pose.pose.position.z = odom.position(2);

    odom_msg.twist.twist.linear.x = odom.velocity(0);
    odom_msg.twist.twist.linear.y = odom.velocity(1);
    odom_msg.twist.twist.linear.z = odom.velocity(2);

    odom_msg.pose.pose.orientation.w = odom.q.w();
    odom_msg.pose.pose.orientation.x = odom.q.x();
    odom_msg.pose.pose.orientation.y = odom.q.y();
    odom_msg.pose.pose.orientation.z = odom.q.z();

    odom_ned_publisher.publish(odom_msg);

}


void Topic_handler::topic_handler_init(ros::NodeHandle& nh) {
    //odom_subscriber = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 100, boost::bind(&Odom::feed, &(this->odom), _1));
    odom_subscriber = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 100, &Topic_handler::odom_callback, this);
    rc_subscriber = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, boost::bind(&RC::feed, &(this->rc), _1));
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, boost::bind(&Imu::feed, &(this->imu), _1));
    mav_cmd_publisher = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    debug_flag = 1;
    odom_ned_publisher = nh.advertise<nav_msgs::Odometry>("/odom/ned", 10);
    pos_pub = nh.advertise<geometry_msgs::Vector3>("/debug/pos", 10);
    vel_pub = nh.advertise<geometry_msgs::Vector3>("/debug/vel", 10);
    cmd_pub = nh.advertise<geometry_msgs::Vector3>("/debug/cmd_rate", 10);
    imu_linear_acc_pub = nh.advertise<geometry_msgs::Vector3>("/debug/imu_linear_acc", 10);
    imu_angle_vel = nh.advertise<geometry_msgs::Vector3>("/debug/imu_angle_vel", 10);

    pos_error = nh.advertise<geometry_msgs::Vector3>("/debug/pos_error", 10);
    vel_error = nh.advertise<geometry_msgs::Vector3>("/debug/vel_error", 10);
    cmd_error = nh.advertise<geometry_msgs::Vector3>("/debug/cmd_error", 10);

    yaw_config = nh.advertise<geometry_msgs::Vector3>("/debug/yaw_config", 10);

    // rc_pub = nh.advertise<std_msgs::UInt16MultiArray>("/debug/rc_out", 10);

}
