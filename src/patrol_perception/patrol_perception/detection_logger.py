#!/usr/bin/env python3
"""Detection logger: records every processed camera frame, every detection, and snapshots.

Subscribes: /detections (JSON from the YOLO node), /detections/image (annotated picture),
            /patrol/state (JSON from the patrol manager).
Writes to ~/detection_runs/det_<timestamp>/:
  frames.csv      one row per processed frame (robot pose in the map frame, patrol lap/waypoint)
  detections.csv  one row per detection (class, raw_class, confidence, box, robot pose)
  snapshots/      annotated picture saved when something is detected (per-class cooldown)
Run with the venv Python (needs Pillow). Matching against ground truth is done afterwards
by tools/analyze_detections.py.
"""
import csv
import json
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

FRAME_FIELDS = ['sim_time', 'frame', 'robot_x', 'robot_y', 'robot_yaw', 'pose_exact',
                'patrol_state', 'lap', 'waypoint', 'n_detections', 'infer_ms']
DET_FIELDS = ['sim_time', 'frame', 'class', 'raw_class', 'conf', 'x1', 'y1', 'x2', 'y2',
              'robot_x', 'robot_y', 'robot_yaw', 'lap', 'waypoint', 'snapshot']


def stamp_key(t):
    return int(round(t * 1000.0))


class DetectionLogger(Node):
    def __init__(self):
        super().__init__('detection_logger')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('snapshot_cooldown_s', 3.0)
        self.declare_parameter('save_snapshots', True)

        base = os.path.expanduser(self.get_parameter('output_dir').value or '~/detection_runs')
        self.run_dir = os.path.join(base, 'det_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.snap_dir = os.path.join(self.run_dir, 'snapshots')
        os.makedirs(self.snap_dir, exist_ok=True)
        self.cooldown = float(self.get_parameter('snapshot_cooldown_s').value)
        self.save_snapshots = bool(self.get_parameter('save_snapshots').value)

        self.f_frames = open(os.path.join(self.run_dir, 'frames.csv'), 'w', newline='')
        self.w_frames = csv.DictWriter(self.f_frames, fieldnames=FRAME_FIELDS)
        self.w_frames.writeheader()
        self.f_dets = open(os.path.join(self.run_dir, 'detections.csv'), 'w', newline='')
        self.w_dets = csv.DictWriter(self.f_dets, fieldnames=DET_FIELDS)
        self.w_dets.writeheader()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.patrol = {'state': 'UNKNOWN', 'lap': '', 'waypoint': ''}
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/patrol/state', self.on_state, state_qos)
        self.create_subscription(String, '/detections', self.on_det, 10)
        self.create_subscription(Image, '/detections/image', self.on_img, 1)

        self.frame_idx = 0
        self.n_dets = 0
        self.n_snaps = 0
        self.pending = {}          # stamp key -> (path, wall time)
        self.last_snap = {}        # class -> sim time of last snapshot
        self.pil_ok = True
        self.get_logger().info(f'Logging detections to {self.run_dir}')

    def on_state(self, msg):
        try:
            d = json.loads(msg.data)
            self.patrol = {'state': d.get('state', ''), 'lap': d.get('lap', ''),
                           'waypoint': d.get('waypoint', '')}
        except ValueError:
            pass

    def robot_pose(self, stamp):
        sec = int(stamp)
        candidates = ((Time(seconds=sec, nanoseconds=int((stamp - sec) * 1e9)), True),
                      (Time(), False))
        for t, exact in candidates:
            try:
                tr = self.tf_buffer.lookup_transform('map', 'base_footprint', t)
            except Exception:
                continue
            q = tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return tr.transform.translation.x, tr.transform.translation.y, yaw, exact
        return None

    def on_det(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        stamp = float(d['stamp'])
        dets = d.get('detections', [])
        self.frame_idx += 1
        pose = self.robot_pose(stamp)
        rx, ry, ryaw, exact = pose if pose else ('', '', '', False)
        rnd = (lambda v: round(v, 3)) if pose else (lambda v: v)

        snap_name = ''
        if dets and self.save_snapshots:
            due = [x['class'] for x in dets
                   if stamp - self.last_snap.get(x['class'], -1e9) >= self.cooldown]
            if due:
                for c in due:
                    self.last_snap[c] = stamp
                names = '-'.join(sorted(set(due)))
                snap_name = f'snapshots/{stamp:.1f}_L{self.patrol["lap"]}_{self.patrol["waypoint"]}_{names}.png'
                self.pending[stamp_key(stamp)] = (os.path.join(self.run_dir, snap_name), time.time())

        self.w_frames.writerow({
            'sim_time': round(stamp, 3), 'frame': self.frame_idx,
            'robot_x': rnd(rx), 'robot_y': rnd(ry), 'robot_yaw': rnd(ryaw), 'pose_exact': exact,
            'patrol_state': self.patrol['state'], 'lap': self.patrol['lap'],
            'waypoint': self.patrol['waypoint'], 'n_detections': len(dets),
            'infer_ms': d.get('infer_ms', '')})
        for x in dets:
            x1, y1, x2, y2 = x['bbox']
            self.w_dets.writerow({
                'sim_time': round(stamp, 3), 'frame': self.frame_idx, 'class': x['class'],
                'raw_class': x.get('raw_class', x['class']), 'conf': x['conf'],
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'robot_x': rnd(rx), 'robot_y': rnd(ry), 'robot_yaw': rnd(ryaw),
                'lap': self.patrol['lap'], 'waypoint': self.patrol['waypoint'],
                'snapshot': snap_name})
            self.n_dets += 1
        self.f_frames.flush()
        self.f_dets.flush()

    def on_img(self, msg):
        key = stamp_key(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        item = self.pending.pop(key, None)
        now = time.time()
        for k in [k for k, v in self.pending.items() if now - v[1] > 10.0]:
            del self.pending[k]
        if item is None or not self.pil_ok:
            return
        try:
            from PIL import Image as PILImage
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            arr = arr[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            PILImage.fromarray(arr, 'RGB').save(item[0])
            self.n_snaps += 1
        except ImportError:
            self.get_logger().error('Pillow is not installed in this Python; snapshots disabled')
            self.pil_ok = False

    def shutdown(self):
        info = {'frames': self.frame_idx, 'detections': self.n_dets, 'snapshots': self.n_snaps,
                'snapshot_cooldown_s': self.cooldown}
        with open(os.path.join(self.run_dir, 'logger_info.yaml'), 'w') as f:
            yaml.safe_dump(info, f, sort_keys=False)
        for f in (self.f_frames, self.f_dets):
            try:
                f.close()
            except Exception:
                pass
        self.get_logger().info(f'Logger finished: {info}')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
