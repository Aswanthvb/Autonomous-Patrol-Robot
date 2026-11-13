#!/usr/bin/env python3
"""Patrol manager: sends one Nav2 goal at a time, loops over waypoints, logs results.

States: WAIT_NAV2 -> SEND_GOAL -> WAIT_ACCEPT -> NAVIGATING -> PAUSE (at waypoint) -> next
A failed, rejected or timed-out waypoint is logged and the patrol moves on.
The manager never drives the motors itself; it only sends goals to Nav2.
"""
import csv
import json
import math
import os
import shutil
import time
from datetime import datetime

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

WAIT_NAV2, SEND_GOAL, WAIT_ACCEPT, NAVIGATING, PAUSE, DONE = (
    'WAIT_NAV2', 'SEND_GOAL', 'WAIT_ACCEPT', 'NAVIGATING', 'PAUSE', 'DONE')

EVENT_FIELDS = [
    'sim_time', 'wall_time', 'lap', 'waypoint_index', 'waypoint', 'result', 'detail',
    'duration_s', 'error_code', 'recoveries', 'target_x', 'target_y',
    'robot_x', 'robot_y', 'robot_yaw']
LAP_FIELDS = [
    'lap', 'waypoints_total', 'waypoints_succeeded', 'lap_success',
    'start_sim_time', 'duration_s']


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        default_wp = os.path.join(
            get_package_share_directory('patrol_task_manager'),
            'config', 'patrol_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_wp)
        self.declare_parameter('laps', -1)            # -1 = use the value in the file
        self.declare_parameter('output_dir', '')
        self.declare_parameter('startup_delay_s', 5.0)

        wp_file = self.get_parameter('waypoints_file').value or default_wp
        with open(wp_file) as f:
            cfg = yaml.safe_load(f)['patrol']
        self.waypoints = cfg['waypoints']
        laps_param = int(self.get_parameter('laps').value)
        self.max_laps = int(cfg.get('laps', 3)) if laps_param < 0 else laps_param
        self.timeout = float(cfg.get('waypoint_timeout_s', 90.0))
        self.pause = float(cfg.get('pause_s', 3.0))
        self.startup_delay = float(self.get_parameter('startup_delay_s').value)

        out_base = os.path.expanduser(
            self.get_parameter('output_dir').value or cfg.get('output_dir', '~/patrol_runs'))
        self.run_dir = os.path.join(out_base, 'run_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(self.run_dir, exist_ok=True)
        shutil.copy(wp_file, os.path.join(self.run_dir, 'waypoints_used.yaml'))
        self.ev_file = open(os.path.join(self.run_dir, 'waypoint_events.csv'), 'w', newline='')
        self.ev_writer = csv.DictWriter(self.ev_file, fieldnames=EVENT_FIELDS)
        self.ev_writer.writeheader()
        self.ev_file.flush()
        self.lap_file = open(os.path.join(self.run_dir, 'lap_summary.csv'), 'w', newline='')
        self.lap_writer = csv.DictWriter(self.lap_file, fieldnames=LAP_FIELDS)
        self.lap_writer.writeheader()
        self.lap_file.flush()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(String, 'patrol/state', state_qos)

        self.state = WAIT_NAV2
        self.finished = False
        self.summary_written = False
        self.wait_start = None
        self.last_wait_log = 0.0
        self.last_pub = 0.0
        self.lap = 0
        self.idx = 0
        self.current_wp_name = ''
        self.lap_start = 0.0
        self.lap_results = []
        self.goal_start = 0.0
        self.goal_handle = None
        self.send_future = None
        self.result_future = None
        self.last_fb = None
        self.timed_out = False
        self.cancel_sent = False
        self.pause_until = 0.0
        self.attempts = 0
        self.successes = 0
        self.laps_done = 0
        self.laps_ok = 0
        self.lap_durations = []

        self.get_logger().info(
            f'Patrol: {len(self.waypoints)} waypoints, laps={self.max_laps or "forever"}, '
            f'timeout={self.timeout:.0f}s, pause={self.pause:.0f}s, output={self.run_dir}')
        self.timer = self.create_timer(0.5, self.tick)

    # ------------------------------------------------------------ helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def make_pose(self, wp):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(wp['x'])
        p.pose.position.y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def on_feedback(self, msg):
        self.last_fb = msg.feedback

    def publish_state(self):
        name = {PAUSE: 'AT_WAYPOINT', SEND_GOAL: 'PATROLLING', WAIT_ACCEPT: 'PATROLLING',
                NAVIGATING: 'PATROLLING', WAIT_NAV2: 'IDLE', DONE: 'DONE'}[self.state]
        msg = String()
        msg.data = json.dumps({
            'state': name, 'lap': self.lap, 'waypoint': self.current_wp_name,
            'index': self.idx, 'sim_time': round(self.now_s(), 2)})
        self.state_pub.publish(msg)

    # ------------------------------------------------------------ main loop
    def tick(self):
        now = self.now_s()
        if now < 1e-3:
            return                      # still waiting for /clock
        if self.state == WAIT_NAV2:
            self.tick_wait(now)
        elif self.state == SEND_GOAL:
            self.send_goal(now)
        elif self.state == WAIT_ACCEPT:
            self.tick_accept(now)
        elif self.state == NAVIGATING:
            self.tick_navigating(now)
        elif self.state == PAUSE:
            if now >= self.pause_until:
                self.advance(now)
        if now - self.last_pub >= 1.0:
            self.last_pub = now
            self.publish_state()

    def tick_wait(self, now):
        if self.wait_start is None:
            self.wait_start = now
        if now - self.wait_start < self.startup_delay:
            return
        ready = self.client.server_is_ready() and self.robot_pose() is not None
        if not ready:
            if now - self.last_wait_log >= 10.0:
                self.last_wait_log = now
                self.get_logger().info('Waiting for Nav2 (navigate_to_pose) and map->base_footprint ...')
            return
        self.get_logger().info('Nav2 is ready. Starting patrol.')
        self.lap = 1
        self.idx = 0
        self.lap_start = now
        self.lap_results = []
        self.state = SEND_GOAL
        self.publish_state()

    def send_goal(self, now):
        wp = self.waypoints[self.idx]
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(wp)
        self.current_wp_name = wp['name']
        self.goal_start = now
        self.timed_out = False
        self.cancel_sent = False
        self.last_fb = None
        self.get_logger().info(
            f'Lap {self.lap}: going to {wp["name"]} ({wp["x"]}, {wp["y"]}) '
            f'[{self.idx + 1}/{len(self.waypoints)}]')
        self.send_future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        self.state = WAIT_ACCEPT

    def tick_accept(self, now):
        if not self.send_future.done():
            if now - self.goal_start > 15.0:
                self.finish_waypoint(now, 'failed', 'goal not accepted in time', '')
            return
        gh = self.send_future.result()
        if gh is None or not gh.accepted:
            self.finish_waypoint(now, 'failed', 'goal rejected by Nav2', '')
            return
        self.goal_handle = gh
        self.result_future = gh.get_result_async()
        self.state = NAVIGATING

    def tick_navigating(self, now):
        elapsed = now - self.goal_start
        if self.result_future.done():
            res = self.result_future.result()
            code = getattr(res.result, 'error_code', '')
            if res.status == GoalStatus.STATUS_SUCCEEDED:
                self.finish_waypoint(now, 'succeeded', 'goal reached', code)
            elif res.status == GoalStatus.STATUS_CANCELED:
                self.finish_waypoint(now, 'cancelled', 'timeout' if self.timed_out else 'cancelled', code)
            else:
                self.finish_waypoint(now, 'failed', f'Nav2 aborted (error {code})', code)
            return
        if elapsed > self.timeout and not self.cancel_sent:
            self.get_logger().warn(f'{self.current_wp_name}: time limit {self.timeout:.0f}s reached, cancelling')
            self.cancel_sent = True
            self.timed_out = True
            self.goal_handle.cancel_goal_async()
        elif self.cancel_sent and elapsed > self.timeout + 20.0:
            self.finish_waypoint(now, 'cancelled', 'timeout (no cancel response)', '')

    # ------------------------------------------------------------ results
    def write_event(self, now, result, detail, code):
        wp = self.waypoints[self.idx]
        pose = self.robot_pose()
        fb = self.last_fb
        row = {
            'sim_time': round(now, 2), 'wall_time': round(time.time(), 2),
            'lap': self.lap, 'waypoint_index': self.idx, 'waypoint': wp['name'],
            'result': result, 'detail': detail,
            'duration_s': round(now - self.goal_start, 2), 'error_code': code,
            'recoveries': getattr(fb, 'number_of_recoveries', '') if fb is not None else '',
            'target_x': wp['x'], 'target_y': wp['y'],
            'robot_x': round(pose[0], 3) if pose else '',
            'robot_y': round(pose[1], 3) if pose else '',
            'robot_yaw': round(pose[2], 3) if pose else ''}
        self.ev_writer.writerow(row)
        self.ev_file.flush()
        self.get_logger().info(
            f'Lap {self.lap} {wp["name"]}: {result.upper()} ({detail}) in {row["duration_s"]}s')

    def finish_waypoint(self, now, result, detail, code):
        self.write_event(now, result, detail, code)
        self.attempts += 1
        ok = result == 'succeeded'
        self.successes += int(ok)
        self.lap_results.append(ok)
        self.goal_handle = None
        if ok and self.pause > 0:
            self.pause_until = now + self.pause
            self.state = PAUSE
        else:
            self.advance(now)

    def advance(self, now):
        self.idx += 1
        if self.idx < len(self.waypoints):
            self.state = SEND_GOAL
            return
        self.end_lap(now)

    def end_lap(self, now):
        total = len(self.lap_results)
        good = sum(self.lap_results)
        success = total > 0 and good == total
        duration = now - self.lap_start
        self.lap_writer.writerow({
            'lap': self.lap, 'waypoints_total': total, 'waypoints_succeeded': good,
            'lap_success': success, 'start_sim_time': round(self.lap_start, 2),
            'duration_s': round(duration, 2)})
        self.lap_file.flush()
        self.laps_done += 1
        self.laps_ok += int(success)
        self.lap_durations.append(duration)
        self.get_logger().info(
            f'Lap {self.lap} finished: {good}/{total} waypoints reached in {duration:.1f}s '
            f'({"SUCCESS" if success else "NOT all reached"})')
        if self.max_laps > 0 and self.lap >= self.max_laps:
            self.finish_run()
            return
        self.lap += 1
        self.idx = 0
        self.lap_start = now
        self.lap_results = []
        self.state = SEND_GOAL

    def write_summary(self, interrupted):
        if self.summary_written:
            return
        self.summary_written = True
        avg = sum(self.lap_durations) / len(self.lap_durations) if self.lap_durations else None
        summary = {
            'interrupted': interrupted,
            'laps_completed': self.laps_done,
            'laps_fully_successful': self.laps_ok,
            'lap_completion_rate': round(self.laps_ok / self.laps_done, 3) if self.laps_done else None,
            'waypoint_attempts': self.attempts,
            'waypoint_successes': self.successes,
            'waypoint_success_rate': round(self.successes / self.attempts, 3) if self.attempts else None,
            'average_lap_time_s': round(avg, 1) if avg is not None else None,
        }
        with open(os.path.join(self.run_dir, 'run_summary.yaml'), 'w') as f:
            yaml.safe_dump(summary, f, sort_keys=False)
        self.get_logger().info(f'Run summary: {summary}')

    def finish_run(self):
        self.state = DONE
        self.publish_state()
        self.write_summary(interrupted=False)
        self.get_logger().info(f'Patrol complete. Logs in {self.run_dir}')
        self.finished = True

    def shutdown(self):
        if self.state == NAVIGATING and self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
                self.write_event(self.now_s(), 'cancelled', 'interrupted by user', '')
            except Exception:
                pass
        if not self.summary_written:
            self.write_summary(interrupted=not self.finished)
        for f in (self.ev_file, self.lap_file):
            try:
                f.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
