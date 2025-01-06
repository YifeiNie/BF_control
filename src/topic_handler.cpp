#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>

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


void RC::feed(mavros_msgs::RCInConstPtr msg){
    roll = msg->channels[0];
    pitch = msg->channels[1];
    yaw = msg->channels[2];
    thrust = msg->channels[3];
    if (msg->channels[5] > 1700){
        is_armed = 1;
    }else{
        is_armed = 0;
    }
    if (msg->channels[7] > 1700){
        is_offboard = 1;
    }else{
        is_offboard = 0;
    }
    rcv_stamp = ros::Time::now();
}


void Odom::feed(nav_msgs::OdometryConstPtr msg){
    position(0) = msg->pose.pose.position.x;
    position(1) = msg->pose.pose.position.y;
    position(2) = msg->pose.pose.position.z;

    velocity(0) = msg->twist.twist.linear.x;
    velocity(1) = msg->twist.twist.linear.y;
    velocity(2) = msg->twist.twist.linear.z;

    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    rcv_stamp = ros::Time::now();
}

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
}
  
double Imu::get_current_yaw(){
    double yaw = atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}

void Topic_handler::topic_handler_init(ros::NodeHandle& nh, Topic_handler& th) {
    odom_subscriber = nh.subscribe<nav_msgs::Odometry>("odom", 100, boost::bind(&Odom::feed, th.odom, _1));
    rc_subscriber = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 100, boost::bind(&RC::feed, th.rc, _1));
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, boost::bind(&Imu::feed, th.imu, _1));
    mav_cmd_publisher = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
}