#pragma once
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>


class Odom_input {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond q;
    Eigen::Vector3d att;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool recv_new_msg;

    ros::Subscriber odom_data_subscriber;

    void feed(nav_msgs::OdometryConstPtr msg);
};

class RC_input {
public:
    
    ros::Subscriber rc_data_subscriber;
    void feed(mavros_msgs::RCIn msg);
};




