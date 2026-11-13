#!/usr/bin/env python3
"""Grab one frame from /camera/image_raw and print everything YOLO sees at very low confidence.

Usage (sim running, backpack in view of the camera):
    source ~/patrol_ws/install/setup.bash
    python3 yolo_debug_frame.py                      # tests yolov8n, yolov8s, yolov8m
    python3 yolo_debug_frame.py yolov8n.pt           # tests only the model(s) you name

Reads the output like this: if 'backpack' (or 'handbag' / 'suitcase') appears with a score
below your node's threshold, the threshold or class filter is the problem, not the model.
Larger models download their weights the first time, so the VM needs internet for that.
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO

MODELS = sys.argv[1:] or ["yolov8n.pt", "yolov8s.pt", "yolov8m.pt"]


class GrabOne(Node):
    def __init__(self):
        super().__init__("yolo_debug_grab")
        self.frame = None
        self.create_subscription(Image, "/camera/image_raw", self.on_image,
                                 qos_profile_sensor_data)

    def on_image(self, msg):
        if self.frame is None:
            # No cv_bridge (it clashes with NumPy 2 in the venv). Ultralytics expects BGR.
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if msg.encoding not in ("rgb8", "bgr8"):
                raise RuntimeError(f"Unsupported encoding: {msg.encoding}")
            img = img.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
            if msg.encoding == "rgb8":
                img = img[:, :, ::-1]
            self.frame = np.ascontiguousarray(img)
            print(f"Got frame {msg.width}x{msg.height}, encoding {msg.encoding}")


def main():
    rclpy.init()
    node = GrabOne()
    print("Waiting for a frame on /camera/image_raw ...")
    while rclpy.ok() and node.frame is None:
        rclpy.spin_once(node, timeout_sec=1.0)
    cv2.imwrite("/tmp/yolo_debug_frame.png", node.frame)
    print("Saved the frame to /tmp/yolo_debug_frame.png (open it to confirm colors look right)")

    for name in MODELS:
        model = YOLO(name)
        result = model(node.frame, conf=0.01, verbose=False)[0]
        rows = sorted(((float(b.conf), result.names[int(b.cls)]) for b in result.boxes),
                      reverse=True)[:8]
        print(f"\n== {name} (top detections, conf >= 0.01) ==")
        if not rows:
            print("  nothing at all, even at 0.01")
        for score, label in rows:
            print(f"  {label:<14} {score:.2f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
