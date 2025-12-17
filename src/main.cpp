#include <ros/ros.h>
#include "BFcontrol_FSM.h"
#include "topic_handler.h"
#include <PID_controller.h>

Topic_handler th;       // 实时更新并存储话题数据

int main(int argc, char **argv){
    ros::init(argc, argv, "offboard_node");

    ros::NodeHandle nh("~");
    BFcontrol_FSM fsm;      // 用于执行程序主体
    th.topic_handler_init(nh);
    fsm.pid.init(nh);
    ros::Rate rate(CTRL_FREQUENCY);
    //ROS_INFO("offboard_node is running");
    while (ros::ok()){
        rate.sleep();
        ros::spinOnce();
    //     if (!th.imu.imu_yaw_offset_initialized &&
    //     th.is_imu_received(ros::Time::now()) &&
    //     th.is_odom_received(ros::Time::now())) {

    //     th.imu.imu_yaw_offset = th.imu.get_current_yaw() - th.odom.get_current_yaw();
    //     th.imu.imu_yaw_offset_initialized = true;

    //     ROS_INFO_STREAM("IMU yaw offset initialized: " << th.imu.imu_yaw_offset * 180.0 / M_PI << " deg");
    // }
        fsm.run(th);
        // th.update_yaw_offset();
        
    }
    return 0;
}