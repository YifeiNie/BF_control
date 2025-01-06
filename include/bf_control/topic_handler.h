#pragma once
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>


class Odom {
public:
    
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond q;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    void feed(nav_msgs::OdometryConstPtr msg);
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
    
    Eigen::Vector3d linear_acc;
    Eigen::Vector3d angle_vel;
    Eigen::Quaterniond q;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;
    double get_current_yaw();
    void feed(sensor_msgs::ImuConstPtr msg);
};



class Topic_handler {
public:
    Odom odom;
    RC rc;
    Imu imu;

    ros::Subscriber odom_subscriber;
    ros::Subscriber rc_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Publisher mav_cmd_publisher; 
    bool is_rc_received(const ros::Time &now_time);
    bool is_odom_received(const ros::Time &now_time);
    bool is_imu_received(const ros::Time &now_time);


    void topic_handler_init(ros::NodeHandle &nh);
};

