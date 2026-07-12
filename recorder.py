#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from odom_recorder import OdomRecorder


def spin_executor(executor):

    executor.spin()


def main():

    rclpy.init()

    recorder = OdomRecorder()

    executor = MultiThreadedExecutor()

    executor.add_node(recorder)

    spin_thread = threading.Thread(
        target=spin_executor,
        args=(executor,),
        daemon=True
    )

    spin_thread.start()

    print()
    print("=" * 65)
    print("        OriginCar Trajectory Recorder")
    print("=" * 65)
    print()
    print("Topic : /odom_combined")
    print()
    print("操作流程：")
    print("1. 启动底盘")
    print("2. 启动 teleop_twist_keyboard")
    print("3. 保持小车静止")
    print("4. 按 Enter 建立原点")
    print("5. 开始遥控小车")
    print("6. Ctrl+C结束录制")
    print()

    input(">>> 小车放到起点后按 Enter 开始录制...")

    recorder.set_origin()

    print()
    print("========== Recording ==========")
    print("Ctrl+C 停止录制")
    print()

    try:

        while rclpy.ok():

            state = recorder.status()

            print(
                "\r"
                f"Points:{state['sample_count']:5d}   "
                f"Distance:{state['travel_distance']:6.2f}m   "
                f"X:{state['x']:7.3f}   "
                f"Y:{state['y']:7.3f}",
                end="",
                flush=True
            )

            time.sleep(0.20)

    except KeyboardInterrupt:

        print()

        print("\nStopping Recorder...")

    finally:

        recorder.stop()

        recorder.save()

        recorder.destroy_node()

        executor.shutdown()

        rclpy.shutdown()

        print()

        print("=" * 65)
        print("Trajectory Saved Successfully.")
        print("=" * 65)


if __name__ == "__main__":

    main()