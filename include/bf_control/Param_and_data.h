#pragma once

#include <ros/ros.h>


struct Gain{
    float Kp_x, Kp_y, Kp_z;
    float Ki_x, Ki_y, Ki_z;
    float Kd_x, Kd_y, Kd_z;

    float Kp_vx, Kp_vy, Kp_vz;
    float Ki_vx, Ki_vy, Ki_vz;
    float Kd_vx, Kd_vy, Kd_vz;
};

struct Msg_timeout{
    double odom;
    double rc;
    double cmd;
    double imu;
};

struct Auto_takeoffLand{
    bool enable;
    bool enable_auto_arm;
    bool no_RC;
    double height;
    double speed;
};

struct RC_reverse{
    int roll;
    int pitch;
    int yaw;
    int throttle;
};

class Static_param {
public: 

    float max_manual_vel;
    float ctrl_freq_max;

    RC_reverse rc_Reverse;
    Auto_takeoffLand auto_takeoff_land;
    Msg_timeout  msg_timeout;

    template <typename TName, typename TVal>
    void get_param(const ros::NodeHandle &nh, const TName &name, TVal &val);
};




