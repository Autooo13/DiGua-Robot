#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================
QR Code Scanner Node (Standalone)
qr_scanner.py

Platform : ROS2 Humble / RDK X5
用途     : 配合 path_follower.py 在循迹过程中扫二维码

功能
----
1. 订阅相机话题（默认 /aurora/rgb/image_raw）
2. 用 OpenCV QRCodeDetector 持续检测二维码
3. 解码结果去重（3 秒内相同内容不重复输出）
4. 输出到终端（ROS logger）
5. 发布到 /sign 话题供其他节点消费

启动方式
--------
python3 qr_scanner.py --ros-args \
  -p image_topic:=/aurora/rgb/image_raw \
  -p decode_every_n_frames:=3

依赖
----
opencv-python (cv2)
cv_bridge
=============================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class QrScanner(Node):
    """独立的二维码扫描节点 — 直接读取相机图像，不依赖 YOLO."""

    def __init__(self):
        super().__init__("qr_scanner")

        # ---- 参数 ----------------------------------------------
        self.declare_parameter("image_topic", "/aurora/rgb/image_raw")
        self.declare_parameter("qr_topic", "/sign")
        self.declare_parameter("decode_every_n_frames", 3)
        self.declare_parameter("dedup_sec", 3.0)

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._qr_topic = str(self.get_parameter("qr_topic").value)
        self._decimate = max(1, int(self.get_parameter("decode_every_n_frames").value))
        self._dedup_sec = float(self.get_parameter("dedup_sec").value)

        self._frame_count = 0
        self._last_text = ""
        self._last_time = 0.0

        # ---- OpenCV -------------------------------------------
        if not HAS_CV:
            self.get_logger().fatal("opencv-python / cv_bridge not installed!")
            raise RuntimeError("Missing cv2 / cv_bridge")

        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()

        # ---- ROS ----------------------------------------------
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(String, self._qr_topic, 10)
        self.create_subscription(Image, self._image_topic, self._on_image, qos_sensor)

        self.get_logger().info(
            f"qr_scanner ready: image_topic={self._image_topic}, "
            f"qr_topic={self._qr_topic}, decimate={self._decimate}"
        )

    # ============================================================
    def _on_image(self, msg: Image) -> None:
        self._frame_count += 1
        if self._frame_count % self._decimate != 0:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"image decode failed: {e}")
            return

        text = self._decode(frame)
        if not text:
            return

        if self._is_duplicate(text):
            return

        self._publish(text)

    # ============================================================
    def _decode(self, frame) -> str:
        try:
            text, _, _ = self._detector.detectAndDecode(frame)
        except Exception as e:
            self.get_logger().warn(f"qr decode error: {e}")
            return ""
        return str(text).strip()

    # ============================================================
    def _is_duplicate(self, text: str) -> bool:
        now = self._now_sec()
        if text == self._last_text and now - self._last_time < self._dedup_sec:
            return True
        return False

    # ============================================================
    def _publish(self, text: str) -> None:
        now = self._now_sec()
        self._last_text = text
        self._last_time = now

        # 发布到 /sign
        msg = String()
        msg.data = text
        self._pub.publish(msg)

        # 终端输出
        # 判断奇偶，输出方向提示
        direction = ""
        try:
            val = int(text)
            if 1 <= val <= 9999:
                direction = " → 顺时针" if val % 2 == 1 else " → 逆时针"
        except ValueError:
            pass

        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  二维码识别结果: {text}{direction}\n"
            f"{'='*50}"
        )

    # ============================================================
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = QrScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
