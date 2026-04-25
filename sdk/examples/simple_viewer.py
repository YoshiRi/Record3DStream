#!/usr/bin/env python3
"""
Simple OpenCV viewer for iPhone sensor streaming.

Usage:
    python simple_viewer.py <iphone_ip>          # WiFi mode
    python simple_viewer.py 192.168.1.100        # WiFi mode
    python simple_viewer.py --usb                # USB mode

Controls:
    Q: Quit
    S: Save current frame
"""

import sys
import os
import argparse
import cv2
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sdk import IPhoneSensorClient


def get_screen_size() -> tuple[int, int]:
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        w, h = root.winfo_screenwidth(), root.winfo_screenheight()
        root.destroy()
        return w, h
    except Exception:
        return 1920, 1080


def colorize_depth(depth: np.ndarray, max_depth: float = 5.0) -> np.ndarray:
    depth_normalized = np.clip(depth / max_depth, 0, 1)
    return cv2.applyColorMap(
        (depth_normalized * 255).astype(np.uint8),
        cv2.COLORMAP_JET
    )


def fit_to_screen(img: np.ndarray, screen_w: int, screen_h: int) -> np.ndarray:
    h, w = img.shape[:2]
    scale = min((screen_w - 20) / w, (screen_h - 60) / h, 1.0)
    if scale < 1.0:
        new_w, new_h = int(w * scale), int(h * scale)
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return img


def draw_info_panel(img: np.ndarray, frame) -> None:
    """Draw camera intrinsics, IMU, and pose info as an overlay on the left edge."""
    intr = frame.intrinsics

    lines = [
        f"Frame: {frame.frame_id}   t={frame.timestamp:.2f}s",
        "",
        "--- Intrinsics (RGB) ---",
        f"  fx={intr.fx:.1f}  fy={intr.fy:.1f}",
        f"  cx={intr.ppx:.1f}  cy={intr.ppy:.1f}",
        f"  {intr.width}x{intr.height}",
    ]

    if frame.imu is not None:
        imu = frame.imu
        lines += [
            "",
            "--- IMU ---",
            f"  accel x={imu.accel[0]:+.2f}",
            f"        y={imu.accel[1]:+.2f}",
            f"        z={imu.accel[2]:+.2f} m/s²",
            f"  gyro  x={imu.gyro[0]:+.3f}",
            f"        y={imu.gyro[1]:+.3f}",
            f"        z={imu.gyro[2]:+.3f} rad/s",
        ]

    t = frame.transform
    lines += [
        "",
        "--- Camera Pose (cam→world) ---",
        f"  pos ({t[0,3]:+.2f}, {t[1,3]:+.2f}, {t[2,3]:+.2f})",
        f"  R0  ({t[0,0]:+.2f}, {t[0,1]:+.2f}, {t[0,2]:+.2f})",
        f"  R1  ({t[1,0]:+.2f}, {t[1,1]:+.2f}, {t[1,2]:+.2f})",
        f"  R2  ({t[2,0]:+.2f}, {t[2,1]:+.2f}, {t[2,2]:+.2f})",
    ]

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.45
    thickness = 1
    line_h = 18
    pad = 8

    text_w = max(cv2.getTextSize(l, font, scale, thickness)[0][0] for l in lines if l)
    panel_w = text_w + pad * 2
    panel_h = line_h * len(lines) + pad * 2

    # Semi-transparent dark background
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (panel_w, panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

    for i, line in enumerate(lines):
        if not line:
            continue
        y = pad + line_h * i + line_h - 4
        color = (180, 220, 255) if line.startswith("---") else (255, 255, 255)
        cv2.putText(img, line, (pad, y), font, scale, color, thickness, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="iPhone Sensor Viewer")
    parser.add_argument("host", nargs="?", default="localhost",
                        help="iPhone IP address (default: localhost for USB)")
    parser.add_argument("--port", "-p", type=int, default=8888,
                        help="Server port (default: 8888)")
    parser.add_argument("--usb", action="store_true",
                        help="Use USB mode (auto-manages iproxy)")
    args = parser.parse_args()

    screen_w, screen_h = get_screen_size()
    print(f"Screen size: {screen_w}x{screen_h}")

    mode_str = "USB" if args.usb else f"{args.host}:{args.port}"
    print(f"Connecting via {mode_str}...")
    client = IPhoneSensorClient(args.host, args.port, usb=args.usb)

    if not client.start():
        print("Failed to connect!")
        sys.exit(1)

    print("Connected! Press Q to quit, S to save frame.")

    while True:
        frame = client.wait_for_frame(timeout=1.0)

        if frame is None:
            if not client.is_connected:
                print("Disconnected!")
                break
            continue

        # Colorize depth and upscale to match color resolution
        depth_vis = colorize_depth(frame.depth)
        depth_vis = cv2.rotate(depth_vis, cv2.ROTATE_90_CLOCKWISE)
        color_rotated = cv2.rotate(frame.color, cv2.ROTATE_90_CLOCKWISE)

        # Upscale depth to match color height
        target_h = color_rotated.shape[0]
        depth_scale = target_h / depth_vis.shape[0]
        depth_upscaled = cv2.resize(
            depth_vis,
            (int(depth_vis.shape[1] * depth_scale), target_h),
            interpolation=cv2.INTER_NEAREST,
        )

        combined = np.hstack([color_rotated, depth_upscaled])
        combined = fit_to_screen(combined, screen_w, screen_h)

        draw_info_panel(combined, frame)

        cv2.imshow("iPhone Sensor Stream", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"frame_{frame.frame_id:06d}"
            cv2.imwrite(f"{filename}_color.jpg", frame.color)
            cv2.imwrite(f"{filename}_depth.png", (frame.depth * 1000).astype(np.uint16))
            np.save(f"{filename}_transform.npy", frame.transform)
            print(f"Saved {filename}_*")

    client.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
