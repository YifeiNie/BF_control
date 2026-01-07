#pragma once
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <std_msgs/UInt16MultiArray.h>
class Odom {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond q;
    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    double yaw_offset = 0.0;
    bool yaw_offset_initialized = false;
    double get_current_yaw() const;
    void feed(nav_msgs::OdometryConstPtr msg);
    Eigen::Vector3d t265_offset = Eigen::Vector3d::Zero();
    bool offset_initialized = false;
    Eigen::Vector3d desired_origin{-0.02, 0.0, -0.06};  // 飞控原点
};

class RC {
public:
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t thrust;
    uint16_t is_offboard;
    uint16_t is_armed;
    ros::Time rcv_stamp;
    void feed(mavros_msgs::RCInConstPtr msg);
};

class Imu {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d linear_acc;
    Eigen::Vector3d angle_vel;
    Eigen::Quaterniond q;
    
    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;
    double get_current_yaw();
    double get_current_yaw(double t265_yaw, bool use_offset); // 新函数

    double imu_yaw_offset = 0.0; // 新增成员变量
    bool imu_yaw_offset_initialized = false;
    void feed(sensor_msgs::ImuConstPtr msg);


};



class Topic_handler {
public:
    Odom odom;
    RC rc;
    Imu imu;


    ros::Publisher odom_ned_publisher;
    ros::Subscriber odom_subscriber;
    ros::Subscriber rc_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Publisher mav_cmd_publisher; 
    bool is_rc_received(const ros::Time &now_time);
    bool is_odom_received(const ros::Time &now_time);
    bool is_imu_received(const ros::Time &now_time);
    bool debug_flag;

    void topic_handler_init(ros::NodeHandle& nh);
    void update_yaw_offset();
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);

    ros::Publisher pos_pub, vel_pub, cmd_pub, imu_linear_acc_pub, imu_angle_vel, pos_error, vel_error, cmd_error, yaw_config;//debug信息

    ros::Publisher rc_pub;

};

