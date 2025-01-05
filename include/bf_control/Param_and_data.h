#pragma once

#include <ros/ros.h>

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




