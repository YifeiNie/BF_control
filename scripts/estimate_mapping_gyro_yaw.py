#!/usr/bin/env python3
"""
estimate_mapping_gyro_yaw.py

订阅：
 - /mavros/imu/data         (sensor_msgs/Imu)   -> FC IMU (fused)
 - /camera/gyro/sample      (sensor_msgs/Imu)   -> T265 raw gyro
或  /camera/odom/sample     (nav_msgs/Odometry) -> use twist.twist.angular for odom rates

功能：
 - 同步收集两边 angular_velocity 向量（或仅 z），并做最小二乘估计线性映射：
    imu_rates ≈ A * cam_rates  (A 是 3x3 映射矩阵)
 - 也对 yaw（通过对角速度积分）做 1D 线性拟合 imu_yaw ≈ s * odom_yaw + b
 - 打印映射矩阵 A、z轴缩放 s 和偏置 b（度）
 - 以便你在代码中修正 yaw 提取或对 odom 取反/偏移

使用：
  chmod +x estimate_mapping_gyro_yaw.py
  source /opt/ros/noetic/setup.bash
  ./estimate_mapping_gyro_yaw.py

按 Ctrl-C 停止并查看最终估计结果。
"""
import rospy, math, numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from collections import deque
from tf.transformations import euler_from_quaternion

# 配置：使用 camera 的哪个 topic（gyro 或 odom.twist）
USE_CAMERA_GYRO = True  # True -> /camera/gyro/sample (Imu); False -> use /camera/odom/sample twist.angular
CAM_GYRO_TOPIC = "/camera/gyro/sample"
CAM_ODOM_TOPIC = "/camera/odom/sample"
FC_IMU_TOPIC = "/mavros/imu/data"

# 缓存大小
MAX_SAMPLES = 2000

cam_buf = deque(maxlen=MAX_SAMPLES)
fc_buf  = deque(maxlen=MAX_SAMPLES)
odom_yaw_buf = deque(maxlen=MAX_SAMPLES)
imu_yaw_buf  = deque(maxlen=MAX_SAMPLES)

# 简单积分器用于得到 yaw estimate from z rate
def integrate_rates(rate_list, dt_list):
    if len(rate_list) < 2:
        return None
    s = 0.0
    yaw_series = []
    for r,dt in zip(rate_list, dt_list):
        s += r * dt
        yaw_series.append(s)
    return yaw_series

# callbacks
last_cam_t = None
last_fc_t = None

def cam_imu_cb(msg):
    global last_cam_t
    t = msg.header.stamp.to_sec()
    av = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=float)
    cam_buf.append((t, av))
    # for yaw integration
    if last_cam_t is not None:
        dt = t - last_cam_t
        cam_rate_history.append((av[2], dt))
    last_cam_t = t

def cam_odom_cb(msg):
    global last_cam_t
    t = msg.header.stamp.to_sec()
    av = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z], dtype=float)
    cam_buf.append((t, av))
    # integrate z
    if last_cam_t is not None:
        dt = t - last_cam_t
        cam_rate_history.append((av[2], dt))
    last_cam_t = t

def fc_cb(msg):
    global last_fc_t
    t = msg.header.stamp.to_sec()
    av = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=float)
    fc_buf.append((t, av))
    # integrate z
    if last_fc_t is not None:
        dt = t - last_fc_t
        fc_rate_history.append((av[2], dt))
    last_fc_t = t

# 全局历史用于积分
cam_rate_history = []
fc_rate_history = []

def sync_and_estimate():
    # Build arrays by nearest-time matching
    if len(cam_buf) < 10 or len(fc_buf) < 10:
        return None
    cam_arr_times = np.array([t for t,_ in cam_buf])
    cam_arr_vals  = np.array([v for _,v in cam_buf])
    fc_arr_times  = np.array([t for t,_ in fc_buf])
    fc_arr_vals   = np.array([v for _,v in fc_buf])

    # for each cam sample find nearest fc sample
    paired_cam = []
    paired_fc  = []
    for i,ct in enumerate(cam_arr_times):
        idx = np.argmin(np.abs(fc_arr_times - ct))
        if abs(fc_arr_times[idx] - ct) < 0.02:  # 20 ms tolerance
            paired_cam.append(cam_arr_vals[i])
            paired_fc.append(fc_arr_vals[idx])
    if len(paired_cam) < 20:
        return None
    X = np.array(paired_cam)  # Nx3
    Y = np.array(paired_fc)   # Nx3

    # Solve Y ≈ X * A^T  => we want A such that Y = X @ A.T
    # Flatten: for each column separately, do least squares
    A = np.zeros((3,3))
    for col in range(3):
        # solve X * a = Y[:,col]
        a,_,_,_ = np.linalg.lstsq(X, Y[:,col], rcond=None)
        A[col,:] = a  # mapping from cam -> fc for this component
    # A: rows index fc-component, columns index cam-component? Let's present mapping as: Y ≈ X @ A.T
    return A, len(paired_cam)

def estimate_yaw_map_from_integration():
    # integrate recent rate histories to get approximate yaw series
    # convert histories to arrays
    if len(cam_rate_history) < 10 or len(fc_rate_history) < 10:
        return None
    # build cumulative series for same time base approximately by sampling min length
    # we will take first N entries
    N = min(len(cam_rate_history), len(fc_rate_history))
    cam_rates = np.array([r for r,dt in cam_rate_history[-N:]])
    cam_dts   = np.array([dt for r,dt in cam_rate_history[-N:]])
    fc_rates  = np.array([r for r,dt in fc_rate_history[-N:]])
    fc_dts    = np.array([dt for r,dt in fc_rate_history[-N:]])
    # integrate to yaw (relative)
    cam_yaw = np.cumsum(cam_rates * cam_dts)
    fc_yaw  = np.cumsum(fc_rates  * fc_dts)
    # unwrap
    cam_yaw_u = np.unwrap(cam_yaw)
    fc_yaw_u  = np.unwrap(fc_yaw)
    # lstsq for s and b: fc_yaw ≈ s * cam_yaw + b
    A = np.vstack([cam_yaw_u, np.ones_like(cam_yaw_u)]).T
    s,b = np.linalg.lstsq(A, fc_yaw_u, rcond=None)[0]
    return s,b, len(cam_yaw_u)

if __name__ == "__main__":
    rospy.init_node("estimate_map_gyro_yaw", anonymous=True)
    if USE_CAMERA_GYRO:
        rospy.Subscriber(CAM_GYRO_TOPIC, Imu, cam_imu_cb)
    else:
        rospy.Subscriber(CAM_ODOM_TOPIC, Odometry, cam_odom_cb)
    rospy.Subscriber(FC_IMU_TOPIC, Imu, fc_cb)
    rate = rospy.Rate(1.0)
    print("Collecting data... move/rotate UAV a few seconds")
    try:
        while not rospy.is_shutdown():
            res = sync_and_estimate()
            if res is not None:
                A, n = res
                print("Linear mapping matrix A (fc ≈ cam @ A.T) samples:", n)
                print(np.round(A,4))
            res2 = estimate_yaw_map_from_integration()
            if res2 is not None:
                s,b,n = res2
                print("Yaw integration fit: fc_yaw ≈ s * cam_yaw + b (deg)")
                print("  s = {:.4f}, b = {:.2f} deg, samples={}".format(s, b*180.0/math.pi, n))
            rate.sleep()
    except KeyboardInterrupt:
        print("Stopped.")
