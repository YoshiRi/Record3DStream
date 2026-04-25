# CLAUDE.md — Record3DStream Development Guide

## Purpose

This repo is a **sensor driver** for iPhone LiDAR/RGBD/IMU. Downstream consumers are ROS2 nodes, SLAM, and CV pipelines. Correctness of timestamps and low/predictable latency are more important than throughput.

## Improvement Plan

Discovered by codebase audit. Implemented in order: SDK → ROS2 driver.

### Commit 1 — SDK: queue-based delivery + packet validation [DONE]

**Files**: `sdk/sdk/client.py`, `sdk/sdk/protocol.py`

| # | Problem | Fix |
|---|---------|-----|
| 1a | Single `_latest_frame` + threading.Event has a race condition and prevents multi-consumer use | Replace with `queue.Queue(maxsize=2)` using drop-oldest on full |
| 1b | No frame statistics (FPS, drop rate) | Add `get_stats()` → `{frames_received, frames_dropped, fps, drop_rate}` |
| 1c | Corrupt packet header can claim arbitrarily large sizes, causing OOM spin | Add bounds check in `get_packet_size()` and `parse_frame()` before size calculation |

### Commit 2 — ROS2: correct timestamps + event-driven receive [DONE]

**Files**: `ros2-driver/ros2_driver/iphone_sensor_node.py`

| # | Problem | Fix |
|---|---------|-----|
| 2a | `create_timer(1/30)` polls; up to 33ms latency between frame arrival and publish | Replace timer with a dedicated `_receive_thread` that blocks on `client.wait_for_frame()` |
| 2b | `self.get_clock().now()` timestamps all messages at *publish* time, not *capture* time | Establish ARKit→ROS2 time offset at first frame; use `frame.timestamp` for all headers |
| 2c | IMU stamp uses publish time; breaks IMU preintegration in SLAM | Use `frame.imu.timestamp` (ARKit monotonic) with the same offset for IMU header |

### Commit 3 — ROS2: heavy processing off the receive path [DONE]

**Files**: `ros2-driver/ros2_driver/iphone_sensor_node.py`

| # | Problem | Fix |
|---|---------|-----|
| 3a | Bilateral filter (~30ms) + EMA + erosion block the receive thread every 5 frames | Move all heavy ops into a `_heavy_worker` thread, fed by `queue.Queue(maxsize=1)` |
| 3b | Point cloud/aligned depth throttled to 1/5 frames by `_heavy_interval` | Worker runs at its natural pace; drop-if-busy queue provides implicit rate limiting without blocking the receive path |
| 3c | Color downsampled to depth resolution with INTER_LINEAR (aliasing at boundaries) | Switch to INTER_AREA for downsampling |

### Future (not yet implemented)

- **Packet CRC / framing**: corrupted mid-packet data silently produces wrong 3D points
- **Depth intrinsics origin**: document whether the protocol sends RGB or depth intrinsics; the current workaround (scale at `get_depth_intrinsics()`) amplifies errors 7.5×
- **Covariance**: CameraInfo `D` field assumes zero distortion; real ARKit LiDAR has some; measure and fill in
- **Per-pixel depth uncertainty**: encode ARKit confidence into PointCloud2 as an additional field for downstream weighting

## Architecture Constraints

- `frame.timestamp` and `frame.imu.timestamp` are both ARKit `CACurrentMediaTime()` (monotonic, seconds). They share the same clock; a single offset converts both to ROS2 time.
- Depth and RGB are already optically aligned by ARKit. `get_aligned_depth()` is purely a resize (INTER_NEAREST); no reprojection needed.
- Protocol v1 has no IMU or confidence. Check `frame.imu is not None` before accessing.

## Development Notes

- SDK changes are pure Python; no ROS2 needed to test.
- ROS2 driver requires Ubuntu + ROS2 Jazzy. Test timestamp drift by `ros2 topic echo /imu` and comparing `header.stamp` against wall clock.
- Keep `_heavy_queue` at `maxsize=1`. Larger values buffer stale point clouds and make latency unpredictable.
