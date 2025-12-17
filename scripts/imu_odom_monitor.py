#!/usr/bin/env python3
# import rospy
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# import math

# def quat_to_euler(qx, qy, qz, qw):
#     roll  = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))
#     pitch = math.asin(2*(qw*qy - qz*qx))
#     yaw   = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
#     return roll, pitch, yaw

# def imu_cb(msg):
#     r, p, y = quat_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
#     print(f"[IMU] Roll: {math.degrees(r):.2f}°, Pitch: {math.degrees(p):.2f}°, Yaw: {math.degrees(y):.2f}°")

# def odom_cb(msg):
#     q = msg.pose.pose.orientation
#     r, p, y = quat_to_euler(q.x, q.y, q.z, q.w)
#     print(f"[Odometry] Roll: {math.degrees(r):.2f}°, Pitch: {math.degrees(p):.2f}°, Yaw: {math.degrees(y):.2f}°\n")

# rospy.init_node('imu_odom_monitor')
# rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
# rospy.Subscriber("/camera/odom/sample", Odometry, odom_cb)
# rospy.spin()



import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf

class ImuOdomMonitor:
    def __init__(self):
        rospy.init_node("imu_odom_monitor")

        # 订阅 topic_handler 转换后的 NED Odom
        self.odom_sub = rospy.Subscriber("/odom/ned", Odometry, self.odom_cb)

        # 订阅飞控 IMU（NED）
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)

        self.odom_yaw = None
        self.imu_yaw = None

    def odom_cb(self, msg):
        # 提取四元数
        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # euler[2] 是 YAW
        self.odom_yaw = euler[2]

    def imu_cb(self, msg):
        # 提取 IMU 四元数
        q = msg.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = euler[2]

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.imu_yaw is not None:
                print("\n------------------")
                print("IMU YAW  (rad): %.3f" % self.imu_yaw)
                print("Odom YAW (rad): %.3f" % self.odom_yaw)
                print("Yaw Diff  (deg): %.2f" % ((self.odom_yaw - self.imu_yaw) * 180 / 3.14159))
            rate.sleep()


if __name__ == "__main__":
    monitor = ImuOdomMonitor()
    monitor.run()
