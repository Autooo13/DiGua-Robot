#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================
OriginCar Trajectory Follower
path_follower.py

Platform : ROS2 Humble
配套     : odom_recorder.py / recorder.py

功能
----
1. 读取 odom_recorder.py 生成的 waypoints.csv (局部坐标系: x, y, yaw)
2. 订阅 /odom_combined，用与录制脚本完全一致的方式建立局部坐标系
   (小车放回录制起点，按 Enter 建立原点)
3. Pure Pursuit 前视点追踪，发布 /cmd_vel 让小车重走该路径
=============================================================
"""

import os
import csv
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# ============================================================
# 工具函数
# ============================================================

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ============================================================
# PathFollower Node
# ============================================================

class PathFollower(Node):

    def __init__(self):

        super().__init__("path_follower")

        # =====================================================
        # 参数
        # =====================================================

        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("waypoint_file", "data/waypoints.csv")

        self.declare_parameter("lookahead_distance", 0.20)   # m, 前视距离
        self.declare_parameter("goal_tolerance", 0.05)       # m, 终点判定容差
        self.declare_parameter("max_linear_speed", 0.15)     # m/s
        self.declare_parameter("min_linear_speed", 0.04)     # m/s, 转弯时不至于停死
        self.declare_parameter("max_angular_speed", 1.0)     # rad/s
        self.declare_parameter("k_angular", 1.8)             # 角速度比例增益
        self.declare_parameter("control_rate", 20.0)         # Hz

        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.waypoint_file = self.get_parameter("waypoint_file").value

        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.k_angular = float(self.get_parameter("k_angular").value)
        self.control_rate = float(self.get_parameter("control_rate").value)

        # =====================================================
        # 世界 / 局部位姿 (与 odom_recorder.py 完全一致的定义)
        # =====================================================

        self.current_world_x = 0.0
        self.current_world_y = 0.0
        self.current_world_yaw = 0.0

        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.origin_initialized = False

        self.local_x = 0.0
        self.local_y = 0.0
        self.local_yaw = 0.0

        # =====================================================
        # 路径 & 状态
        # =====================================================

        self.waypoints = []          # [(x, y, yaw), ...]
        self.target_index = 0
        self.finished = False
        self.following = False

        # =====================================================
        # ROS 接口
        # =====================================================

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # 控制指令用独立定时器发布，不依赖 odom 回调频率。
        # 很多底盘驱动对 /cmd_vel 有看门狗超时（收不到新指令就自动刹停），
        # 如果 /odom_combined 频率较低（比如 2Hz），绑在 odom 回调里发指令
        # 会导致车"走一下停一下"甚至看起来像直接停住。
        self.create_timer(
            1.0 / self.control_rate,
            self.control_step
        )

        self.get_logger().info(f"Subscribe : {self.odom_topic}")
        self.get_logger().info(f"Publish   : {self.cmd_vel_topic}")
        self.get_logger().info(f"Control Rate : {self.control_rate} Hz")

    # ============================================================
    # 加载路径点
    # ============================================================

    def load_waypoints(self):

        if not os.path.exists(self.waypoint_file):
            raise FileNotFoundError(
                f"waypoint file not found: {self.waypoint_file}"
            )

        waypoints = []

        with open(self.waypoint_file, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append((
                    float(row["x"]),
                    float(row["y"]),
                    float(row["yaw"])
                ))

        if len(waypoints) == 0:
            raise ValueError("waypoint file is empty")

        self.waypoints = waypoints
        self.target_index = 0
        self.finished = False

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}"
        )

    # ============================================================
    # 建立原点 (与录制时保持一致: 车放回起点后按 Enter)
    # ============================================================

    def set_origin(self):

        self.origin_x = self.current_world_x
        self.origin_y = self.current_world_y
        self.origin_yaw = self.current_world_yaw
        self.origin_initialized = True

        self.get_logger().info("Origin Created.")

    # ============================================================
    # Odom 回调
    # ============================================================

    def odom_callback(self, msg):

        pose = msg.pose.pose

        self.current_world_x = pose.position.x
        self.current_world_y = pose.position.y
        self.current_world_yaw = quaternion_to_yaw(pose.orientation)

        if not self.origin_initialized:
            return

        dx = self.current_world_x - self.origin_x
        dy = self.current_world_y - self.origin_y
        theta = -self.origin_yaw

        self.local_x = math.cos(theta) * dx - math.sin(theta) * dy
        self.local_y = math.sin(theta) * dx + math.cos(theta) * dy
        self.local_yaw = normalize_angle(self.current_world_yaw - self.origin_yaw)

    # ============================================================
    # Pure Pursuit 控制核心
    # ============================================================

    def control_step(self):

        # 现在由定时器周期性调用，需要自行判断是否应该发指令
        if not self.following or not self.origin_initialized:
            return

        if self.finished or len(self.waypoints) == 0:
            return

        # ---- 找前视目标点：从当前索引开始，找第一个距离 >= 前视距离的点 ----
        idx = self.target_index
        last_idx = len(self.waypoints) - 1

        while idx < last_idx:
            wx, wy, _ = self.waypoints[idx]
            dist = math.hypot(wx - self.local_x, wy - self.local_y)
            if dist >= self.lookahead_distance:
                break
            idx += 1

        self.target_index = idx
        tx, ty, tyaw = self.waypoints[idx]

        dx = tx - self.local_x
        dy = ty - self.local_y
        distance_to_target = math.hypot(dx, dy)

        # ---- 终点判定：走到最后一个点且距离足够近 ----
        if idx == last_idx and distance_to_target < self.goal_tolerance:
            self.stop_and_finish()
            return

        # ---- 计算航向误差 ----
        angle_to_target = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_target - self.local_yaw)

        # ---- 角速度 (比例控制 + 限幅) ----
        angular_z = self.k_angular * heading_error
        angular_z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_z)
        )

        # ---- 线速度：航向误差越大，线速度越小；接近终点减速 ----
        heading_factor = max(0.0, math.cos(heading_error))
        linear_x = self.max_linear_speed * heading_factor

        if idx == last_idx:
            # 接近终点时按剩余距离做减速斜坡
            slow_down = max(
                self.min_linear_speed,
                min(1.0, distance_to_target / max(self.lookahead_distance, 1e-6))
            )
            linear_x *= slow_down

        if heading_factor > 0.05:
            linear_x = max(linear_x, self.min_linear_speed)
        else:
            # 航向偏差过大，原地转向，不前进
            linear_x = 0.0

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        print(
            "\r"
            f"WP:{idx+1:4d}/{len(self.waypoints):4d}   "
            f"X:{self.local_x:7.3f}  Y:{self.local_y:7.3f}   "
            f"HeadingErr:{math.degrees(heading_error):6.1f}°   "
            f"V:{linear_x:5.2f}  W:{angular_z:5.2f}",
            end="",
            flush=True
        )

    # ============================================================
    # 停止
    # ============================================================

    def stop_and_finish(self):

        self.finished = True
        self.following = False
        self.publish_stop()

        print()
        self.get_logger().info("Path Following Finished.")

    def publish_stop(self):

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # ============================================================
    # 开始 / 暂停
    # ============================================================

    def start_following(self):

        if not self.origin_initialized:
            self.get_logger().warning("Origin not set yet.")
            return

        if len(self.waypoints) == 0:
            self.get_logger().warning("No waypoints loaded.")
            return

        self.following = True
        self.finished = False

    def pause_following(self):

        self.following = False
        self.publish_stop()

    def status(self):

        return {
            "target_index": self.target_index,
            "total": len(self.waypoints),
            "x": self.local_x,
            "y": self.local_y,
            "yaw": self.local_yaw,
            "finished": self.finished,
        }


# ============================================================
# main
# ============================================================

def spin_executor(executor):
    executor.spin()


def main():

    rclpy.init()

    follower = PathFollower()

    executor = MultiThreadedExecutor()
    executor.add_node(follower)

    spin_thread = threading.Thread(
        target=spin_executor,
        args=(executor,),
        daemon=True
    )
    spin_thread.start()

    print()
    print("=" * 65)
    print("        OriginCar Trajectory Follower")
    print("=" * 65)
    print()
    print(f"Waypoint File : {follower.waypoint_file}")
    print(f"Odom Topic    : {follower.odom_topic}")
    print(f"Cmd Vel Topic : {follower.cmd_vel_topic}")
    print()

    try:
        follower.load_waypoints()
    except Exception as e:
        print(f"[ERROR] {e}")
        follower.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        return

    print()
    print("操作流程：")
    print("1. 启动底盘")
    print("2. 将小车放回【录制轨迹时的起点】，并保持与起点相同的朝向")
    print("3. 保持小车静止")
    print("4. 按 Enter 建立原点（必须与录制时的起点一致，否则路径会整体偏移）")
    print("5. 小车将自动沿录制路径行驶，到达终点自动停止")
    print("6. Ctrl+C 可随时中止")
    print()

    input(">>> 小车放到起点后按 Enter 开始循迹...")

    follower.set_origin()
    follower.start_following()

    print()
    print("========== Following ==========")
    print("Ctrl+C 停止")
    print()

    try:
        while rclpy.ok() and not follower.finished:
            time.sleep(0.1)

        if follower.finished:
            time.sleep(0.5)  # 留时间把最后状态打印完整

    except KeyboardInterrupt:
        print()
        print("\nStopping Follower...")

    finally:
        follower.pause_following()
        follower.publish_stop()

        follower.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

        print()
        print("=" * 65)
        print("Trajectory Following Ended.")
        print("=" * 65)


if __name__ == "__main__":
    main()
