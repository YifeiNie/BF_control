
#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>

#include "PID_controller.h"
#include "topic_handler.h"

enum State{
    MANUAL_CTRL, 
    HOVER,
    CMD_CTRL,	
    TAKEOFF,
    LAND
};

class BFcontrol_FSM {
public:
    State state = MANUAL_CTRL;
    PID_controller pid;

    void run(Topic_handler &th);
};
