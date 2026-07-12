#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================
OriginCar Trajectory Recorder V2.1
odom_recorder.py

Author : ChatGPT
Platform : ROS2 Humble

Features
---------
1. Subscribe /odom_combined (EKF)
2. Build local coordinate
3. Distance based sampling
4. Linear interpolation
5. Deadband filter
6. Waypoint generation
7. CSV export
8. Report generation
=============================================================
"""

import os
import csv
import math

from dataclasses import dataclass
from typing import List, Optional

from rclpy.node import Node
from nav_msgs.msg import Odometry


# ============================================================
# Pose2D
# ============================================================

@dataclass
class Pose2D:

    index: int = 0

    stamp: float = 0.0

    x: float = 0.0

    y: float = 0.0

    yaw: float = 0.0

    travel: float = 0.0


# ============================================================
# Recorder
# ============================================================

class OdomRecorder(Node):

    def __init__(self):

        super().__init__("odom_recorder")

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter(
            "odom_topic",
            "/odom_combined"
        )

        self.declare_parameter(
            "sample_distance",
            0.05
        )

        self.declare_parameter(
            "deadband",
            0.002
        )

        self.odom_topic = self.get_parameter(
            "odom_topic"
        ).value

        self.sample_distance = float(

            self.get_parameter(
                "sample_distance"
            ).value

        )

        self.deadband = float(

            self.get_parameter(
                "deadband"
            ).value

        )

        # =====================================================
        # World Pose
        # =====================================================

        self.current_pose = Pose2D()

        self.origin_pose: Optional[Pose2D] = None

        self.origin_initialized = False

        # =====================================================
        # Local Pose
        # =====================================================

        self.local_pose = Pose2D()

        # =====================================================
        # Recorder
        # =====================================================

        self.recording = False

        self.sample_index = 0

        self.total_distance = 0.0

        self.accumulated_distance = 0.0

        self.samples: List[Pose2D] = []

        # =====================================================
        # Last Pose
        # =====================================================

        self.last_world_pose = None

        self.last_local_pose = None

        self.last_sample_pose = None

        # =====================================================
        # Subscriber
        # =====================================================

        self.create_subscription(

            Odometry,

            self.odom_topic,

            self.odom_callback,

            20

        )

        self.get_logger().info(

            f"Subscribe : {self.odom_topic}"

        )

    # ============================================================
    # Utils
    # ============================================================

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(

            math.sin(angle),

            math.cos(angle)

        )

    @staticmethod
    def quaternion_to_yaw(q):

        siny = 2.0 * (

            q.w * q.z +

            q.x * q.y

        )

        cosy = 1.0 - 2.0 * (

            q.y * q.y +

            q.z * q.z

        )

        return math.atan2(

            siny,

            cosy

        )

    # ============================================================
    # Set Origin
    # ============================================================

    def set_origin(self):

        self.origin_pose = Pose2D(

            stamp=self.current_pose.stamp,

            x=self.current_pose.x,

            y=self.current_pose.y,

            yaw=self.current_pose.yaw,

            travel=0.0

        )

        self.origin_initialized = True

        self.recording = True

        self.total_distance = 0.0

        self.accumulated_distance = 0.0

        self.sample_index = 0

        self.samples.clear()

        self.last_world_pose = None

        self.last_local_pose = None

        self.last_sample_pose = None

        self.get_logger().info(

            "Origin Created."

        )

    # ============================================================
    # Restart
    # ============================================================

    def restart(self):

        self.origin_initialized = False

        self.recording = False

        self.total_distance = 0.0

        self.accumulated_distance = 0.0

        self.sample_index = 0

        self.samples.clear()

        self.last_world_pose = None

        self.last_local_pose = None

        self.last_sample_pose = None

        self.get_logger().info(

            "Recorder Restart."

        )

    # ============================================================
    # Pause
    # ============================================================

    def pause(self):

        self.recording = False

        self.get_logger().info(

            "Recording Paused."

        )

    # ============================================================
    # Continue
    # ============================================================

    def resume(self):

        if self.origin_initialized:

            self.recording = True

            self.get_logger().info(

                "Recording Continue."

            )

    # ============================================================
    # Stop
    # ============================================================

    def stop(self):

        self.recording = False

    # ============================================================
    # Current Local Pose
    # ============================================================

    def get_local_pose(self):

        return Pose2D(

            stamp=self.local_pose.stamp,

            x=self.local_pose.x,

            y=self.local_pose.y,

            yaw=self.local_pose.yaw,

            travel=self.total_distance

        )

    # ============================================================
    # Odom Callback
    # ============================================================

    def odom_callback(self, msg):

        pose = msg.pose.pose

        self.current_pose.stamp = (

            msg.header.stamp.sec +

            msg.header.stamp.nanosec * 1e-9

        )

        self.current_pose.x = pose.position.x

        self.current_pose.y = pose.position.y

        self.current_pose.yaw = self.quaternion_to_yaw(

            pose.orientation

        )

        if not self.origin_initialized:

            return

        dx = (

            self.current_pose.x -

            self.origin_pose.x

        )

        dy = (

            self.current_pose.y -

            self.origin_pose.y

        )

        theta = -self.origin_pose.yaw

        self.local_pose.x = (

            math.cos(theta) * dx -

            math.sin(theta) * dy

        )

        self.local_pose.y = (

            math.sin(theta) * dx +

            math.cos(theta) * dy

        )

        self.local_pose.yaw = self.normalize_angle(

            self.current_pose.yaw -

            self.origin_pose.yaw

        )

        self.local_pose.stamp = self.current_pose.stamp

        if self.recording:

            self.update_sampling()

    # ============================================================
    # Add Sample
    # ============================================================

    def add_sample(self, pose: Pose2D):

        sample = Pose2D(

            index=self.sample_index,

            stamp=pose.stamp,

            x=pose.x,

            y=pose.y,

            yaw=pose.yaw,

            travel=self.total_distance

        )

        self.samples.append(sample)

        self.last_sample_pose = sample

        self.sample_index += 1

        print(
            f"[{sample.index:04d}] "
            f"X={sample.x:.3f} "
            f"Y={sample.y:.3f} "
            f"Yaw={math.degrees(sample.yaw):7.2f}° "
            f"Travel={sample.travel:.3f}m"
        )

    # ============================================================
    # Pose Interpolation
    # ============================================================

    def interpolate_pose(
        self,
        start: Pose2D,
        end: Pose2D,
        distance
    ):

        dx = end.x - start.x
        dy = end.y - start.y

        seg = math.hypot(dx, dy)

        if seg < 1e-9:

            return Pose2D(

                stamp=end.stamp,

                x=start.x,

                y=start.y,

                yaw=start.yaw,

                travel=self.total_distance

            )

        ratio = distance / seg

        ratio = max(0.0, min(1.0, ratio))

        x = start.x + dx * ratio

        y = start.y + dy * ratio

        dyaw = self.normalize_angle(

            end.yaw - start.yaw

        )

        yaw = self.normalize_angle(

            start.yaw + dyaw * ratio

        )

        return Pose2D(

            stamp=end.stamp,

            x=x,

            y=y,

            yaw=yaw,

            travel=self.total_distance

        )

    # ============================================================
    # Update Sampling
    # ============================================================

    def update_sampling(self):

        current = self.get_local_pose()

        # --------------------------------------------------------
        # First Pose
        # --------------------------------------------------------

        if self.last_local_pose is None:

            self.last_local_pose = current

            self.last_sample_pose = current

            self.add_sample(current)

            return

        # --------------------------------------------------------
        # Current Segment
        # --------------------------------------------------------

        start = self.last_local_pose
        end = current

        dx = end.x - start.x
        dy = end.y - start.y

        segment_length = math.hypot(dx, dy)

        if segment_length < self.deadband:

            return

        self.total_distance += segment_length

        remain = segment_length

        cursor = Pose2D(

            stamp=start.stamp,

            x=start.x,

            y=start.y,

            yaw=start.yaw,

            travel=self.total_distance

        )

        while remain > 1e-9:

            need = (

                self.sample_distance -

                self.accumulated_distance

            )

            # ----------------------------------------------
            # 当前线段不足一个采样距离
            # ----------------------------------------------

            if remain < need:

                self.accumulated_distance += remain

                break

            # ----------------------------------------------
            # 插值生成Waypoint
            # ----------------------------------------------

            pose = self.interpolate_pose(

                cursor,

                end,

                need

            )

            self.add_sample(pose)

            cursor = pose

            remain -= need

            self.accumulated_distance = 0.0

        self.last_local_pose = current

    # ============================================================
    # Statistics
    # ============================================================

    def get_sample_count(self):

        return len(self.samples)

    def get_total_distance(self):

        return self.total_distance

    def get_end_pose(self):

        return self.get_local_pose()

    def get_end_error(self):

        pose = self.get_local_pose()

        return math.hypot(

            pose.x,

            pose.y

        )

    def has_origin(self):

        return self.origin_initialized

    def is_recording(self):

        return self.recording

    # ============================================================
    # Save CSV
    # ============================================================

    def save(self, save_dir="data"):

        if len(self.samples) == 0:

            self.get_logger().warning(
                "No trajectory recorded."
            )

            return

        os.makedirs(save_dir, exist_ok=True)

        trajectory_file = os.path.join(
            save_dir,
            "trajectory.csv"
        )

        waypoint_file = os.path.join(
            save_dir,
            "waypoints.csv"
        )

        report_file = os.path.join(
            save_dir,
            "report.txt"
        )

        # ======================================================
        # trajectory.csv
        # ======================================================

        with open(
            trajectory_file,
            "w",
            newline="",
            encoding="utf-8"
        ) as f:

            writer = csv.writer(f)

            writer.writerow([
                "index",
                "stamp",
                "x",
                "y",
                "yaw(rad)",
                "yaw(deg)",
                "travel(m)"
            ])

            for p in self.samples:

                writer.writerow([
                    p.index,
                    f"{p.stamp:.6f}",
                    f"{p.x:.6f}",
                    f"{p.y:.6f}",
                    f"{p.yaw:.6f}",
                    f"{math.degrees(p.yaw):.3f}",
                    f"{p.travel:.6f}"
                ])

        # ======================================================
        # waypoint.csv
        # ======================================================

        with open(
            waypoint_file,
            "w",
            newline="",
            encoding="utf-8"
        ) as f:

            writer = csv.writer(f)

            writer.writerow([
                "x",
                "y",
                "yaw"
            ])

            for p in self.samples:

                writer.writerow([
                    f"{p.x:.6f}",
                    f"{p.y:.6f}",
                    f"{p.yaw:.6f}"
                ])

        # ======================================================
        # Report
        # ======================================================

        end_pose = self.get_local_pose()

        end_error = math.hypot(
            end_pose.x,
            end_pose.y
        )

        loop_error = 0.0

        if self.total_distance > 1e-6:

            loop_error = (
                end_error /
                self.total_distance
            ) * 100.0

        with open(
            report_file,
            "w",
            encoding="utf-8"
        ) as f:

            f.write("=========================================\n")
            f.write(" OriginCar Trajectory Recorder Report\n")
            f.write("=========================================\n\n")

            f.write(f"Odom Topic          : {self.odom_topic}\n")
            f.write(f"Sample Distance     : {self.sample_distance:.3f} m\n")
            f.write(f"Deadband            : {self.deadband:.3f} m\n\n")

            f.write(f"Samples             : {len(self.samples)}\n")
            f.write(f"Travel Distance     : {self.total_distance:.3f} m\n\n")

            f.write(f"End X               : {end_pose.x:.4f} m\n")
            f.write(f"End Y               : {end_pose.y:.4f} m\n")
            f.write(f"End Yaw             : {math.degrees(end_pose.yaw):.2f} deg\n")
            f.write(f"End Error           : {end_error:.4f} m\n")
            f.write(f"Loop Error          : {loop_error:.2f} %\n")

        self.print_report()

        self.get_logger().info(
            f"Trajectory saved -> {trajectory_file}"
        )

        self.get_logger().info(
            f"Waypoints saved -> {waypoint_file}"
        )

        self.get_logger().info(
            f"Report saved -> {report_file}"
        )

    # ============================================================
    # Print Report
    # ============================================================

    def print_report(self):

        end = self.get_local_pose()

        end_error = math.hypot(
            end.x,
            end.y
        )

        if self.total_distance > 1e-6:

            loop_error = (
                end_error /
                self.total_distance
            ) * 100.0

        else:

            loop_error = 0.0

        print()
        print("=" * 65)
        print("          OriginCar Trajectory Report")
        print("=" * 65)

        print(f"Odom Topic          : {self.odom_topic}")
        print(f"Sample Distance     : {self.sample_distance:.3f} m")
        print(f"Deadband            : {self.deadband:.3f} m")
        print()

        print(f"Waypoint Number     : {len(self.samples)}")
        print(f"Travel Distance     : {self.total_distance:.3f} m")
        print()

        print(f"End Position X      : {end.x:.4f} m")
        print(f"End Position Y      : {end.y:.4f} m")
        print(f"End Heading         : {math.degrees(end.yaw):.2f} deg")
        print()

        print(f"End Error           : {end_error*100:.2f} cm")
        print(f"Loop Error          : {loop_error:.2f} %")

        print("=" * 65)

    # ============================================================
    # Export Waypoints
    # ============================================================

    def get_waypoints(self):

        return self.samples.copy()

    # ============================================================
    # Clear
    # ============================================================

    def clear(self):

        self.samples.clear()

        self.sample_index = 0

        self.total_distance = 0.0

        self.accumulated_distance = 0.0

        self.last_world_pose = None

        self.last_local_pose = None

        self.last_sample_pose = None

    # ============================================================
    # Status
    # ============================================================

    def status(self):

        pose = self.get_local_pose()

        return {

            "recording": self.recording,

            "origin": self.origin_initialized,

            "sample_count": len(self.samples),

            "travel_distance": self.total_distance,

            "x": pose.x,

            "y": pose.y,

            "yaw": pose.yaw,

            "end_error": math.hypot(
                pose.x,
                pose.y
            )
        }