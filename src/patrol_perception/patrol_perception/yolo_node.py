#!/usr/bin/env python3
"""YOLO detector: /camera/image_raw -> /detections (JSON) and /detections/image (annotated).

Runs in a Python virtual environment (~/yolo_venv). Images are converted with numpy only
(no cv_bridge), and only the newest frame is processed at max_rate_hz, so a slow CPU
never builds up a backlog.
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


# The simulated backpack is seen by COCO-trained YOLO as 'suitcase' (0.65 in testing).
ALIASES = {'suitcase': 'backpack'}


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model_path', os.path.expanduser('~/yolo_models/yolov8n.pt'))
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('classes', ['person', 'backpack', 'suitcase'])
        self.declare_parameter('publish_annotated', True)

        model_path = os.path.expanduser(self.get_parameter('model_path').value)
        if not os.path.exists(model_path):
            raise RuntimeError(f'YOLO weights not found: {model_path}')
        from ultralytics import YOLO          # imported here so a missing install gives a clear error
        self.model = YOLO(model_path)
        wanted = list(self.get_parameter('classes').value)
        self.names = self.model.names
        self.class_ids = [i for i, n in self.names.items() if n in wanted]
        if not self.class_ids:
            raise RuntimeError(f'None of {wanted} found in the model classes')
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.period = 1.0 / max(0.1, float(self.get_parameter('max_rate_hz').value))
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)

        # warm-up so the first real frame is not slow
        self.model.predict(np.zeros((480, 640, 3), np.uint8), verbose=False, imgsz=self.imgsz)

        self.pub_det = self.create_publisher(String, '/detections', 10)
        self.pub_img = self.create_publisher(Image, '/detections/image', 1)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.on_image, qos_profile_sensor_data)

        self.last_t = 0.0
        self.warned_enc = False
        self.n_frames = 0
        self.infer_total = 0.0
        self.last_log = time.monotonic()
        self.get_logger().info(
            f'YOLO ready: classes={[self.names[i] for i in self.class_ids]}, conf={self.conf}, '
            f'max {1.0 / self.period:.1f} Hz')

    def to_bgr(self, msg):
        """sensor_msgs/Image -> HxWx3 uint8 BGR array (the order ultralytics expects)."""
        enc = msg.encoding
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc)
        if channels is None:
            if not self.warned_enc:
                self.get_logger().error(f'Unsupported image encoding: {enc}')
                self.warned_enc = True
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        img = data.reshape(msg.height, msg.step)[:, :msg.width * channels]
        img = img.reshape(msg.height, msg.width, channels)[:, :, :3]
        if enc in ('rgb8', 'rgba8'):
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)

    def on_image(self, msg):
        now = time.monotonic()
        if now - self.last_t < self.period:
            return
        self.last_t = now
        frame = self.to_bgr(msg)
        if frame is None:
            return

        t0 = time.monotonic()
        res = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz,
                                 classes=self.class_ids, verbose=False)[0]
        infer_ms = (time.monotonic() - t0) * 1000.0

        dets = []
        for b in res.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            dets.append({'class': ALIASES.get(self.names[int(b.cls[0])], self.names[int(b.cls[0])]), 'raw_class': self.names[int(b.cls[0])], 'conf': round(float(b.conf[0]), 3),
                         'bbox': [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)]})

        out = String()
        out.data = json.dumps({
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'image_w': msg.width, 'image_h': msg.height,
            'infer_ms': round(infer_ms, 1), 'detections': dets})
        self.pub_det.publish(out)

        if self.publish_annotated and self.pub_img.get_subscription_count() > 0:
            ann = res.plot()[:, :, ::-1]                      # BGR -> RGB
            ann = np.ascontiguousarray(ann)
            im = Image()
            im.header = msg.header
            im.height, im.width = int(ann.shape[0]), int(ann.shape[1])
            im.encoding = 'rgb8'
            im.is_bigendian = 0
            im.step = im.width * 3
            im.data = ann.tobytes()
            self.pub_img.publish(im)

        self.n_frames += 1
        self.infer_total += infer_ms
        if time.monotonic() - self.last_log >= 10.0:
            self.last_log = time.monotonic()
            self.get_logger().info(
                f'{self.n_frames} frames processed, avg inference {self.infer_total / self.n_frames:.0f} ms')


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
