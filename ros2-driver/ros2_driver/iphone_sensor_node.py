#!/usr/bin/env python3
"""ROS2 node for iPhone RGBD streaming.

Publishes RGB, depth, aligned depth, confidence, point cloud, IMU,
and camera info topics. Broadcasts TF frames.
"""

import queue as _queue
import sys
import math
import threading
from typing import Optional

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Imu, LaserScan
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from cv_bridge import CvBridge

from sdk import IPhoneSensorClient
from sdk.frame import Intrinsics


class IPhoneSensorNode(Node):
    def __init__(self):
        super().__init__("iphone_sensor_node")

        # Parameters
        self.declare_parameter("host", "192.168.1.100")
        self.declare_parameter("port", 8888)
        self.declare_parameter("camera_name", "camera")
        self.declare_parameter("publish_pointcloud", True)
        self.declare_parameter("publish_aligned_depth", True)
        self.declare_parameter("publish_confidence", True)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("usb", False)
        self.declare_parameter("depth_range_min", 0.1)
        self.declare_parameter("depth_range_max", 5.0)
        self.declare_parameter("min_confidence", 1)

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.usb_mode = self.get_parameter("usb").value
        self.camera_name = self.get_parameter("camera_name").value
        self.pub_pc_enabled = self.get_parameter("publish_pointcloud").value
        self.pub_aligned_enabled = self.get_parameter("publish_aligned_depth").value
        self.pub_conf_enabled = self.get_parameter("publish_confidence").value
        self.pub_imu_enabled = self.get_parameter("publish_imu").value
        self.pub_scan_enabled = self.get_parameter("publish_scan").value
        self.depth_min = self.get_parameter("depth_range_min").value
        self.depth_max = self.get_parameter("depth_range_max").value
        self.min_confidence = self.get_parameter("min_confidence").value

        # Frame IDs
        self.color_optical_frame = f"{self.camera_name}_color_optical_frame"
        self.depth_optical_frame = f"{self.camera_name}_depth_optical_frame"
        self.laser_frame = f"{self.camera_name}_laser_frame"
        self.camera_link_frame = f"{self.camera_name}_link"
        self.world_frame = "world"

        # QoS: best effort, volatile, keep last 1 — matches RViz2 sensor expectations
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_color = self.create_publisher(Image, "color/image_raw", sensor_qos)
        self.pub_color_info = self.create_publisher(CameraInfo, "color/camera_info", sensor_qos)
        self.pub_depth = self.create_publisher(Image, "depth/image_rect_raw", sensor_qos)
        self.pub_depth_info = self.create_publisher(CameraInfo, "depth/camera_info", sensor_qos)

        if self.pub_aligned_enabled:
            self.pub_aligned = self.create_publisher(Image, "aligned_depth_to_color/image_raw", sensor_qos)
            self.pub_aligned_info = self.create_publisher(CameraInfo, "aligned_depth_to_color/camera_info", sensor_qos)

        if self.pub_pc_enabled:
            self.pub_pointcloud = self.create_publisher(PointCloud2, "depth/color/points", sensor_qos)

        if self.pub_conf_enabled:
            self.pub_confidence = self.create_publisher(Image, "confidence/image_raw", sensor_qos)

        if self.pub_imu_enabled:
            self.pub_imu = self.create_publisher(Imu, "imu", sensor_qos)

        if self.pub_scan_enabled:
            self.pub_scan = self.create_publisher(LaserScan, "scan", sensor_qos)

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # CV bridge
        self.bridge = CvBridge()

        # Publish static TFs (camera_link -> optical frames)
        self._publish_static_tfs()

        # Connect to iPhone
        self.client = IPhoneSensorClient(self.host, self.port, usb=self.usb_mode)
        if self.usb_mode:
            self.get_logger().info("Connecting via USB (iproxy)...")
        else:
            self.get_logger().info(f"Connecting via WiFi to {self.host}:{self.port}...")

        if not self.client.start():
            self.get_logger().error("Failed to connect to iPhone!")
            sys.exit(1)

        self.get_logger().info("Connected! Starting frame loop.")

        self._frame_count = 0
        self._log_interval = 30

        # Depth filter parameters (tune to trade stability vs responsiveness)
        self._temporal_alpha = 0.4   # EMA weight of current frame (lower = smoother)
        self._temporal_delta = 0.04  # Reset threshold in metres (preserves fast motion)

        # ARKit → ROS2 time offset, established at the first received frame.
        # frame.timestamp is CACurrentMediaTime() (monotonic, seconds from an
        # arbitrary epoch). We record the difference once so every subsequent
        # message gets a stamp that reflects actual capture time.
        self._time_offset_ns: Optional[int] = None

        # Heavy work queue: bilateral filter + EMA + erosion + point cloud.
        # maxsize=1 means the worker always picks up the latest unprocessed frame
        # and silently drops frames it cannot keep up with — no back-pressure on
        # the receive path.
        self._heavy_queue: _queue.Queue = _queue.Queue(maxsize=1)

        self._node_running = True
        self._heavy_thread = threading.Thread(target=self._heavy_worker, daemon=True)
        self._heavy_thread.start()
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    # ------------------------------------------------------------------
    # Time conversion
    # ------------------------------------------------------------------

    def _ros2_stamp_from_arkit(self, arkit_time: float):
        """Convert an ARKit CACurrentMediaTime timestamp to a ROS2 stamp."""
        stamp_ns = int(arkit_time * 1e9) + self._time_offset_ns
        return Time(nanoseconds=max(0, stamp_ns)).to_msg()

    # ------------------------------------------------------------------
    # Receive thread
    # ------------------------------------------------------------------

    def _receive_loop(self):
        """Dedicated thread: blocks on wait_for_frame(), then publishes.

        Using a thread instead of a ROS2 timer eliminates the up-to-33ms
        polling gap and ensures each frame is published as soon as it arrives.
        """
        while self._node_running and rclpy.ok():
            frame = self.client.wait_for_frame(timeout=1.0)
            if frame is None:
                if not self.client.is_connected:
                    self.get_logger().error("Disconnected from iPhone!")
                    break
                continue

            # Establish the ARKit→ROS2 offset at the first frame.
            if self._time_offset_ns is None:
                ros2_ns = self.get_clock().now().nanoseconds
                self._time_offset_ns = ros2_ns - int(frame.timestamp * 1e9)

            self._process_frame(frame)

    # ------------------------------------------------------------------
    # Per-frame processing (time-critical path: color, depth, IMU, scan)
    # ------------------------------------------------------------------

    def _process_frame(self, frame):
        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info(
                f"First frame: depth={frame.depth.shape}, color={frame.color.shape}, "
                f"conf={'yes' if frame.confidence is not None else 'no'}, "
                f"imu={'yes' if frame.imu is not None else 'no'}"
            )
        elif self._frame_count % self._log_interval == 0:
            stats = self.client.get_stats()
            self.get_logger().info(
                f"Frames: {self._frame_count} | "
                f"SDK fps={stats['fps']} drop={stats['drop_rate']:.1%}"
            )

        # Stamp derived from ARKit capture time, not publish time.
        stamp = self._ros2_stamp_from_arkit(frame.timestamp)
        color_header = Header(stamp=stamp, frame_id=self.color_optical_frame)
        depth_header = Header(stamp=stamp, frame_id=self.depth_optical_frame)

        # Intrinsics before rotation
        rgb_intr = frame.intrinsics
        depth_intr = frame.get_depth_intrinsics()

        # Rotate 90° CW to correct landscape (top-left) → portrait orientation
        frame.color = cv2.rotate(frame.color, cv2.ROTATE_90_CLOCKWISE)
        frame.depth = cv2.rotate(frame.depth, cv2.ROTATE_90_CLOCKWISE)
        if frame.confidence is not None:
            frame.confidence = cv2.rotate(frame.confidence, cv2.ROTATE_90_CLOCKWISE)

        # Update intrinsics for 90° CW rotation:
        # new fx=old fy, new fy=old fx, new cx=old H-1-cy, new cy=old cx
        old_rgb = rgb_intr
        rgb_intr = Intrinsics(
            width=old_rgb.height, height=old_rgb.width,
            fx=old_rgb.fy, fy=old_rgb.fx,
            ppx=old_rgb.height - 1 - old_rgb.ppy, ppy=old_rgb.ppx,
        )
        old_dep = depth_intr
        depth_intr = Intrinsics(
            width=old_dep.height, height=old_dep.width,
            fx=old_dep.fy, fy=old_dep.fx,
            ppx=old_dep.height - 1 - old_dep.ppy, ppy=old_dep.ppx,
        )

        # --- Publish color image ---
        color_msg = self.bridge.cv2_to_imgmsg(frame.color, encoding="bgr8")
        color_msg.header = color_header
        self.pub_color.publish(color_msg)
        self.pub_color_info.publish(self._make_camera_info(color_header, rgb_intr))

        # --- Publish depth image (32FC1 metres) ---
        depth_msg = self.bridge.cv2_to_imgmsg(frame.depth, encoding="32FC1")
        depth_msg.header = depth_header
        self.pub_depth.publish(depth_msg)
        self.pub_depth_info.publish(self._make_camera_info(depth_header, depth_intr))

        # --- Publish confidence ---
        if self.pub_conf_enabled and frame.confidence is not None:
            conf_msg = self.bridge.cv2_to_imgmsg(frame.confidence, encoding="mono8")
            conf_msg.header = depth_header
            self.pub_confidence.publish(conf_msg)

        # --- Publish IMU (use ARKit IMU timestamp, not frame timestamp) ---
        if self.pub_imu_enabled and frame.imu is not None:
            imu_stamp = self._ros2_stamp_from_arkit(frame.imu.timestamp)
            imu_msg = Imu()
            imu_msg.header = Header(stamp=imu_stamp, frame_id=f"{self.camera_name}_imu_frame")
            # ARKit → ROS coordinate transform
            imu_msg.linear_acceleration.x = float(frame.imu.accel[0])
            imu_msg.linear_acceleration.y = float(-frame.imu.accel[2])
            imu_msg.linear_acceleration.z = float(frame.imu.accel[1])
            imu_msg.angular_velocity.x = float(frame.imu.gyro[0])
            imu_msg.angular_velocity.y = float(-frame.imu.gyro[2])
            imu_msg.angular_velocity.z = float(frame.imu.gyro[1])
            imu_msg.orientation_covariance[0] = -1.0           # orientation unknown
            imu_msg.linear_acceleration_covariance[0] = -1.0   # magnitude unknown
            imu_msg.angular_velocity_covariance[0] = -1.0      # magnitude unknown
            self.pub_imu.publish(imu_msg)

        # --- Publish LaserScan ---
        if self.pub_scan_enabled:
            laser_header = Header(stamp=stamp, frame_id=self.laser_frame)
            scan_msg = self._make_laserscan(frame.depth, depth_intr, laser_header)
            if scan_msg is not None:
                self.pub_scan.publish(scan_msg)

        # --- Broadcast ARKit pose as TF: world -> camera_link ---
        self._broadcast_pose(frame.transform, stamp)

        # --- Submit heavy work (non-blocking; drop if worker is busy) ---
        try:
            self._heavy_queue.put_nowait({
                "depth": frame.depth,          # already rotated; worker will copy
                "color": frame.color,
                "confidence": frame.confidence,
                "depth_intr": depth_intr,
                "rgb_intr": rgb_intr,
                "color_header": color_header,
                "depth_header": depth_header,
            })
        except _queue.Full:
            pass  # worker busy; skip heavy processing for this frame

    # ------------------------------------------------------------------
    # Heavy worker thread (bilateral filter, EMA, erosion, point cloud)
    # ------------------------------------------------------------------

    def _heavy_worker(self):
        """Off-thread worker for CPU-intensive depth processing.

        Runs at its natural pace. If the main receive thread submits a new
        task while the worker is busy, the old task is silently dropped
        (queue maxsize=1). This prevents stale point clouds from accumulating
        without stalling the receive path.
        """
        prev_depth = None
        while self._node_running and rclpy.ok():
            try:
                task = self._heavy_queue.get(timeout=1.0)
            except _queue.Empty:
                continue

            depth = task["depth"].copy()
            color = task["color"]
            confidence = task["confidence"]
            depth_intr = task["depth_intr"]
            rgb_intr = task["rgb_intr"]
            color_header = task["color_header"]
            depth_header = task["depth_header"]

            # 1) Bilateral filter: smooth spatial noise, preserve real edges.
            depth = cv2.bilateralFilter(depth, d=5, sigmaColor=0.04, sigmaSpace=4.5)

            # 2) Temporal EMA: reduce per-frame LiDAR jitter (~5-20mm).
            #    Resets at depth discontinuities so real motion responds instantly.
            if prev_depth is not None and prev_depth.shape == depth.shape:
                diff = np.abs(depth - prev_depth)
                blend = (diff < self._temporal_delta) & (depth > 0) & (prev_depth > 0)
                depth = np.where(
                    blend,
                    self._temporal_alpha * depth + (1 - self._temporal_alpha) * prev_depth,
                    depth,
                )
            prev_depth = depth.copy()

            # 3) Valid mask + erosion to remove flying pixels at boundaries.
            valid_mask = (depth > self.depth_min) & (depth < self.depth_max) & np.isfinite(depth)
            if confidence is not None and self.min_confidence > 0:
                valid_mask &= (confidence >= self.min_confidence)
            kernel = np.ones((3, 3), np.uint8)
            valid_mask = cv2.erode(valid_mask.astype(np.uint8), kernel, iterations=1).astype(bool)

            # Publish aligned depth (use filtered depth, not original)
            if self.pub_aligned_enabled:
                aligned = cv2.resize(
                    depth,
                    (color.shape[1], color.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
                aligned_msg = self.bridge.cv2_to_imgmsg(aligned, encoding="32FC1")
                aligned_msg.header = color_header
                self.pub_aligned.publish(aligned_msg)
                self.pub_aligned_info.publish(self._make_camera_info(color_header, rgb_intr))

            # Publish point cloud
            if self.pub_pc_enabled:
                pc_msg = self._make_pointcloud(depth, valid_mask, color, depth_intr, depth_header)
                if pc_msg is not None:
                    self.pub_pointcloud.publish(pc_msg)

    # ------------------------------------------------------------------
    # Message builders
    # ------------------------------------------------------------------

    def _make_camera_info(self, header, intr):
        """Build CameraInfo from Intrinsics."""
        msg = CameraInfo()
        msg.header = header
        msg.width = intr.width
        msg.height = intr.height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            intr.fx, 0.0, intr.ppx,
            0.0, intr.fy, intr.ppy,
            0.0, 0.0, 1.0,
        ]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            intr.fx, 0.0, intr.ppx, 0.0,
            0.0, intr.fy, intr.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def _make_laserscan(self, depth, depth_intr, header):
        """Convert middle row of depth image to LaserScan.

        Depth is in optical frame (Z-forward, X-right).
        LaserScan is in body frame (X-forward, Y-left).
        Mapping: x_body = z_opt, y_body = -x_opt
        Body angle = -atan2(x_opt, z_opt)  (negated from optical)
        """
        height, width = depth.shape
        mid_row = depth[height // 2, :]

        u = np.arange(width, dtype=np.float32)

        angles = -np.arctan2(u - depth_intr.ppx, depth_intr.fx)

        # Reverse so angle_min < angle_max
        angles = angles[::-1]
        mid_row = mid_row[::-1]

        angle_min = float(angles[0])
        angle_max = float(angles[-1])
        angle_increment = float((angle_max - angle_min) / (width - 1))

        cos_angles = np.cos(angles)
        ranges = np.where(cos_angles > 0, mid_row / cos_angles, float('inf'))
        ranges = np.where(
            (ranges >= self.depth_min) & (ranges <= self.depth_max) & np.isfinite(ranges),
            ranges, float('inf')
        ).astype(np.float32)

        msg = LaserScan()
        msg.header = header
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / 30.0
        msg.range_min = float(self.depth_min)
        msg.range_max = float(self.depth_max)
        msg.ranges = ranges.tolist()
        return msg

    def _make_pointcloud(self, depth, valid_mask, color, depth_intr, header):
        """Generate PointCloud2 from pre-filtered depth and color.

        Filtering (bilateral, EMA, erosion) is done before calling this method.
        """
        height, width = depth.shape

        u = np.arange(width, dtype=np.float32)
        v = np.arange(height, dtype=np.float32)
        u, v = np.meshgrid(u, v)

        z = depth
        x = (u - depth_intr.ppx) * z / depth_intr.fx
        y = (v - depth_intr.ppy) * z / depth_intr.fy

        # Downsample color to depth resolution.
        # INTER_AREA reduces aliasing compared to INTER_LINEAR for large
        # downscale ratios (1920x1440 → 256x192, ~7.5×).
        color_resized = cv2.resize(color, (width, height), interpolation=cv2.INTER_AREA)

        b = color_resized[:, :, 0].astype(np.uint32)
        g = color_resized[:, :, 1].astype(np.uint32)
        r = color_resized[:, :, 2].astype(np.uint32)
        rgb_packed = (r << 16) | (g << 8) | b
        rgb_float = rgb_packed.view(np.float32)

        x = x[valid_mask]
        y = y[valid_mask]
        z = z[valid_mask]
        rgb_float = rgb_float[valid_mask]

        n_points = x.shape[0]
        if n_points == 0:
            return None

        points = np.zeros(n_points, dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("rgb", np.float32),
        ])
        points["x"] = x
        points["y"] = y
        points["z"] = z
        points["rgb"] = rgb_float

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n_points
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n_points
        msg.data = points.tobytes()
        msg.is_dense = True
        return msg

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------

    def _publish_static_tfs(self):
        """Publish static TFs: camera_link -> optical frames + laser frame.

        Optical frame convention: X-right, Y-down, Z-forward.
        From camera_link (X-forward, Y-left, Z-up): roll=-90, yaw=-90.
        Laser frame: identity transform from camera_link (X-forward, Z-up).
        """
        now = self.get_clock().now().to_msg()
        transforms = []

        # Rotation: camera_link -> optical frame
        # roll=-90 deg, pitch=0, yaw=-90 deg → q = (x=-0.5, y=0.5, z=-0.5, w=0.5)
        q_optical = (0.5, -0.5, 0.5, -0.5)  # (x, y, z, w) in ROS

        for child_frame in [self.color_optical_frame, self.depth_optical_frame]:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.camera_link_frame
            t.child_frame_id = child_frame
            t.transform.rotation.x = q_optical[0]
            t.transform.rotation.y = q_optical[1]
            t.transform.rotation.z = q_optical[2]
            t.transform.rotation.w = q_optical[3]
            transforms.append(t)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.camera_link_frame
        t.child_frame_id = self.laser_frame
        t.transform.rotation.w = 1.0
        transforms.append(t)

        self.static_tf_broadcaster.sendTransform(transforms)

    def _broadcast_pose(self, transform, stamp):
        """Broadcast ARKit pose as TF: world -> camera_link.

        ARKit: X-right, Y-up, Z-toward-user
        ROS:   X-forward, Y-left, Z-up

        Position: ros = (arkit.x, -arkit.z, arkit.y)
        Orientation: pre-multiply by correction quaternion
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.camera_link_frame

        t.transform.translation.x = float(transform[0, 3])
        t.transform.translation.y = float(-transform[2, 3])
        t.transform.translation.z = float(transform[1, 3])

        R = transform[:3, :3].copy()
        R_ros = np.array([
            [R[0, 0], -R[2, 0], R[1, 0]],
            [-R[0, 2], R[2, 2], -R[1, 2]],
            [R[0, 1], -R[2, 1], R[1, 1]],
        ])

        qw, qx, qy, qz = self._rotation_matrix_to_quaternion(R_ros)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """Convert 3x3 rotation matrix to quaternion (w, x, y, z)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return w, x, y, z

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._node_running = False
        self.client.stop()
        if hasattr(self, "_receive_thread"):
            self._receive_thread.join(timeout=2.0)
        if hasattr(self, "_heavy_thread"):
            self._heavy_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IPhoneSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
