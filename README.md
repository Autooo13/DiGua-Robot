# DiGua-Robot
AI for OriginCar

# 融合扫码的循迹

在不修改你的原始代码（`path_follower.py` / `odom_recorder.py` / `recorder.py`）的前提下，
增加一个**独立的 QR 扫码节点**，与循迹同时运行。

## 文件

```
融合扫码的循迹/
├── qr_scanner.py    ← 独立 QR 扫码节点（新增，不动你的原始文件）
└── README.md        ← 本文件
```

## 设计思路

- **不开 YOLOv5**，直接用 OpenCV 的 `cv2.QRCodeDetector` 读相机画面
- 相机持续开着，每 N 帧解码一次，扫到二维码就输出到终端
- 同时发布到 `/sign` 话题，方便以后其他节点消费
- 与 `path_follower.py` 完全解耦 — 各自独立运行，互不干扰

## 启动方式

按以下顺序在 RDK X5 上启动（每个命令一个终端）：

```bash
# 终端 1: 底盘驱动
ros2 launch origincar_base origincar_bringup.launch.py

# 终端 2: 深度相机（QR 扫码需要 RGB 图像）
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py

# 终端 3: QR 扫码节点
python3 qr_scanner.py --ros-args \
  -p image_topic:=/aurora/rgb/image_raw \
  -p decode_every_n_frames:=3

# 终端 4: 循迹（你的原始 path_follower.py，不动）
python3 path_follower.py
# 小车放回起点，按 Enter 开始循迹
```

## QR 输出效果

小车接近二维码区域时，终端会自动显示：

```
[qr_scanner]: ==================================================
[qr_scanner]:   二维码识别结果: 1234 → 逆时针
[qr_scanner]: ==================================================
```

- 纯数字 1-9999：自动判断方向（奇数顺时针，偶数逆时针）
- 非纯数字文本：直接显示原文
- 3 秒内相同内容不重复输出

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `image_topic` | `/aurora/rgb/image_raw` | 相机 RGB 图像话题 |
| `qr_topic` | `/sign` | QR 结果发布话题 |
| `decode_every_n_frames` | `3` | 每 N 帧解码一次（降低 CPU） |
| `dedup_sec` | `3.0` | 相同内容去重窗口（秒） |

## 你的原始文件

以下文件**完全没有被修改**，保持原样：

| 文件 | 用途 |
|------|------|
| `odom_recorder.py` | 录制轨迹 |
| `recorder.py` | 录制器入口 |
| `path_follower.py` | Pure Pursuit 循迹 |
