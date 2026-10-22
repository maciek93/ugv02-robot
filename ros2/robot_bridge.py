#!/usr/bin/env python3
"""ROS sensor/goal bridge. The Pi guardian remains the sole ESP32 writer."""
import json
import math
import os
from pathlib import Path
import re
import secrets
import threading
import time

from flask import Flask, jsonify, request
from werkzeug.serving import make_server
import requests

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, StaticTransformBroadcaster

from metric_map import MetricGrid, angle_delta
from motion_feedback import AdaptivePWM, wrap_delta


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def quaternion(yaw):
    return 0., 0., math.sin(yaw / 2), math.cos(yaw / 2)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Bridge(Node):
    def __init__(self):
        super().__init__('ugv_metric_bridge')
        keys = json.loads(Path('/etc/ugv-supervised/keys.json').read_text())
        self.key = keys['navigation']
        self.guardian = 'http://127.0.0.1:5000'
        self.lock = threading.RLock()
        self.scan = self.map = self.depth = self.telemetry = None
        self.scan_seq = 0
        self.map_version = None
        self.last_scan_at = self.last_cmd_at = self.last_health_at = 0.
        self.command = (0., 0.)
        self.goal = self.result = None
        self.action_handle = None
        self.goal_seq = 0
        self.odom = [0., 0., 0.]
        self.total_yaw = 0.
        self.last_encoders = None
        self.last_odom_at = None
        self.last_chassis_sample_at = None
        self.wheel_control = AdaptivePWM()
        self.profile_id = None
        self.static_sent = False
        observation_path = Path('/run/ugv-nav/observation-profile.json')
        self.observation_profile = json.loads(observation_path.read_text()) if observation_path.exists() else None
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.depth_sub = self.create_subscription(String, '/tof/grid', self.on_depth, 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.on_velocity, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.path_client = ActionClient(self, FollowPath, '/follow_path')
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.create_timer(.05, self.tick)

    def rpc(self, path, data=None, timeout=.15):
        method = requests.get if data is None else requests.post
        response = method(self.guardian + path, headers={'X-Robot-Key': self.key},
                          json=data, timeout=timeout) if data is not None else method(
                              self.guardian + path, headers={'X-Robot-Key': self.key}, timeout=timeout)
        result = response.json()
        if not response.ok:
            raise PermissionError(result.get('error', 'Guardian rejected request'))
        return result

    def on_scan(self, msg):
        valid = sum(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges)
        with self.lock:
            self.scan = msg
            self.scan_seq += 1
            sequence = self.scan_seq
            self.last_scan_at = time.monotonic()
        # The guardian checks independent, advancing scan evidence, not merely
        # motor lease renewal. Send on a separate thread to keep ROS callbacks free.
        threading.Thread(target=self.send_scan, args=(sequence, msg, valid), daemon=True).start()

    def send_scan(self, sequence, msg, valid):
        try:
            self.rpc('/metric/scan', {'sequence': sequence, 'stamp': stamp_seconds(msg.header.stamp),
                                      'frame_id': msg.header.frame_id, 'valid_beams': valid})
        except Exception:
            pass  # A missing scan heartbeat makes the guardian stop motion.

    def on_map(self, msg):
        info = msg.info
        origin = info.origin
        data = list(msg.data)
        snapshot = {'source': 'ros-occupancy-grid', 'frame_id': msg.header.frame_id,
                    'stamp': stamp_seconds(msg.header.stamp), 'width': info.width,
                    'height': info.height, 'resolution_m': info.resolution,
                    'origin': [origin.position.x, origin.position.y, yaw_of(origin.orientation)],
                    'data': data,
                    'calibration_status': 'observation-only' if self.observation_profile else 'measured-profile'}
        try:
            grid = MetricGrid(snapshot)
        except ValueError as error:
            self.get_logger().warning(f'Rejected map: {error}')
            return
        snapshot['map_version'] = grid.version
        with self.lock:
            self.map = snapshot
            self.map_version = grid.version

    def on_depth(self, msg):
        try:
            depth = json.loads(msg.data)
            if depth.get('sensor') == 'VL53L5CX' and len(depth.get('distance_mm', [])) == 64:
                with self.lock:
                    self.depth = depth
        except (ValueError, TypeError):
            pass

    def on_velocity(self, msg):
        with self.lock:
            self.command = (float(msg.linear.x), float(msg.angular.z))
            self.last_cmd_at = time.monotonic()

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        return {'x': t.transform.translation.x, 'y': t.transform.translation.y,
                'yaw': yaw_of(t.transform.rotation), 'stamp': stamp_seconds(t.header.stamp),
                'frame_id': 'map', 'uncertainty': None}

    def publish_odom(self, telemetry):
        p = telemetry.get('profile') or self.observation_profile
        data = (telemetry.get('chassis') or {}).get('data') or {}
        if not p or data.get('odl') is None or data.get('odr') is None:
            return
        profile_id = ('guardian:' if telemetry.get('profile') else 'observation:') + p['configuration_id']
        if self.profile_id != profile_id:
            self.profile_id = profile_id
            self.last_encoders = None
            self.odom = [0., 0., 0.]
            self.total_yaw = 0.
            self.last_chassis_sample_at = None
            self.wheel_control.reset()
            self.static_sent = False
        if not self.static_sent:
            static = TransformStamped()
            static.header.frame_id = 'base_link'
            static.child_frame_id = 'base_laser'
            static.header.stamp = self.get_clock().now().to_msg()
            static.transform.translation.x = p['lidar_x_m']
            static.transform.translation.y = p['lidar_y_m']
            q = quaternion(p['lidar_yaw_rad'])
            static.transform.rotation.x, static.transform.rotation.y, static.transform.rotation.z, static.transform.rotation.w = q
            self.static_pub.sendTransform(static)
            self.static_sent = True
        now = time.monotonic()
        age = (telemetry.get('chassis') or {}).get('age_s')
        if type(age) not in (int, float) or not 0 <= age <= .75:
            return
        sample_at = now - age
        if self.last_chassis_sample_at is not None and sample_at - self.last_chassis_sample_at < .03:
            return  # The 20 Hz bridge may see the same 5 Hz UART sample repeatedly.
        self.last_chassis_sample_at = sample_at
        counts = (float(data['odl']), float(data['odr']))  # Guardian remaps physical sides.
        if self.last_encoders is None:
            self.last_encoders, self.last_odom_at = counts, sample_at
            return
        dt = sample_at - self.last_odom_at
        left = (counts[0] - self.last_encoders[0]) * p['encoder_m_per_unit'] * p['encoder_signs'][0]
        right = (counts[1] - self.last_encoders[1]) * p['encoder_m_per_unit'] * p['encoder_signs'][1]
        self.last_encoders, self.last_odom_at = counts, sample_at
        if not 0 < dt <= .75 or max(abs(left), abs(right)) > p['upper_speed_m_s'] * dt + .04:
            self.get_logger().warning('Encoder reset or implausible delta; odometry not advanced')
            return
        self.wheel_control.observe(left, right, dt, now)
        distance, turn = (left + right) / 2, (right - left) / p['track_m']
        heading = self.odom[2] + turn / 2
        self.odom[0] += distance * math.cos(heading)
        self.odom[1] += distance * math.sin(heading)
        self.odom[2] = (self.odom[2] + turn + math.pi) % (2*math.pi) - math.pi
        self.total_yaw += turn
        stamp = self.get_clock().now().to_msg()
        q = quaternion(self.odom[2])
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.odom[:2]
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        msg.twist.twist.linear.x = distance / dt
        msg.twist.twist.angular.z = turn / dt
        msg.pose.covariance[0] = msg.pose.covariance[7] = .05
        msg.pose.covariance[35] = .1
        self.odom_pub.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.odom[:2]
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_pub.sendTransform(t)

    def scan_clear(self, turn=False):
        with self.lock:
            scan = self.scan
            p = (self.telemetry or {}).get('profile')
            age = time.monotonic() - self.last_scan_at
        if not scan or not p or age > .5:
            return False
        front = p['length_m']/2 + p['fallback_stop_m'] + p['margin_m']
        side = p['width_m']/2 + p['margin_m']
        radius = math.hypot(p['length_m']/2, p['width_m']/2) + p['margin_m'] + .05
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue
            bearing = scan.angle_min + i * scan.angle_increment + p['lidar_yaw_rad']
            x = p['lidar_x_m'] + distance * math.cos(bearing)
            y = p['lidar_y_m'] + distance * math.sin(bearing)
            if turn and math.hypot(x, y) < radius:
                return False
            if not turn and 0 < x < front and abs(y) < side:
                return False
        return True

    def tick(self):
        now = time.monotonic()
        goal = self.goal
        try:
            telemetry = self.rpc('/metric/telemetry')
            with self.lock:
                self.telemetry = telemetry
                self.last_health_at = now
            self.publish_odom(telemetry)
            if not goal:
                return
            mission = telemetry['mission']['active']
            if not mission or mission['id'] != goal['mission_id']:
                raise PermissionError('Mission authority ended')
            if goal['kind'] == 'direct':
                linear, angular, finished = self.direct_command(goal)
                if finished:
                    self.finish('reached', goal['goal_id'])
                    return
            else:
                with self.lock:
                    linear, angular = self.command if now - self.last_cmd_at < .2 else (0., 0.)
            if (linear > .001 and not self.scan_clear()) or (abs(angular) > .001 and not self.scan_clear(turn=True)):
                self.finish('blocked', goal['goal_id'])
                return
            left, right = self.wheel_control.command(linear, angular, telemetry['profile'], now)
            if self.wheel_control.stalled(now):
                self.get_logger().warning('Wheel barely moving at maximum PWM; stopping goal')
                self.finish('blocked', goal['goal_id'])
                return
            identity = {k: goal[k] for k in ('mission_id', 'session', 'configuration_id', 'goal_id')}
            self.rpc('/metric/navigation/renew', {**identity, 'seq': self.goal_seq,
                                                  'left_pwm': left, 'right_pwm': right})
            self.goal_seq += 1
        except Exception as error:
            if goal:
                self.get_logger().error(f'Navigation stopped: {error}')
                outcome = 'budget_exhausted' if 'budget_exhausted' in str(error) else 'tracking_lost'
                self.finish(outcome, goal['goal_id'])

    def direct_command(self, goal):
        start = goal['odom_start']
        current = self.odom
        if goal['operation'] == 'drive_distance':
            target = goal['value']
            dx, dy = current[0]-start[0], current[1]-start[1]
            travelled = dx*math.cos(start[2]) + dy*math.sin(start[2])
            if travelled >= target - .02:
                return 0., 0., True
            current_pose = self.pose()
            heading_error = (angle_delta(goal['map_start_yaw'], current_pose['yaw'])
                             if current_pose and goal.get('map_start_yaw') is not None else
                             angle_delta(start[2], current[2]))
            cruise = self.telemetry['profile']['forward_m_s_at_pwm']
            return min(cruise, max(.04, (target-travelled)*.6)), max(-.2, min(.2, -heading_error)), False
        target = goal['value']
        direction = 1 if target >= 0 else -1
        turned = direction * (self.total_yaw - goal['yaw_start'])
        current_pose = self.pose()
        if current_pose and goal.get('last_map_yaw') is not None:
            step = wrap_delta(goal['last_map_yaw'], current_pose['yaw'])
            if abs(step) < .5:  # Ignore a localisation jump, not a real 20 Hz turn.
                goal['map_turn_total'] += step
                turned = direction * goal['map_turn_total']
            goal['last_map_yaw'] = current_pose['yaw']
        if turned >= abs(target) - math.radians(3):
            return 0., 0., True
        return 0., direction * min(self.telemetry['profile']['turn_rad_s_at_pwm'], max(.15, (abs(target)-turned)*.7)), False

    def begin(self, kind, body):
        with self.lock:
            if self.goal:
                raise PermissionError('A movement is already running')
            telemetry = self.telemetry
            grid = MetricGrid(self.map) if self.map else None
            pose = self.pose()
        if not telemetry or not grid or not pose or time.monotonic()-self.last_scan_at > .5:
            raise PermissionError('Fresh map, pose and LiDAR scan required')
        if time.time() - grid.stamp > 2 or time.time() - pose['stamp'] > 1.5:
            raise PermissionError('Map or localisation is stale')
        if body.get('map_version') != grid.version:
            raise PermissionError('Map changed; preview again')
        m = telemetry['mission']['active']
        if not m or body.get('mission_id') != m['id']:
            raise PermissionError('Start an approved metric mission')
        goal_id = body.get('goal_id') or secrets.token_hex(12)
        data = {'mission_id': m['id'], 'session': m['session'], 'configuration_id': m['configuration_id'],
                'goal_id': goal_id, 'map_version': grid.version, 'action': kind}
        if kind == 'follow_route':
            body_preview = grid.preview(pose, body['waypoints'], self.footprint(telemetry))
            if body_preview['verdict'] != 'clear':
                raise PermissionError(f"Route preview is {body_preview['verdict']}")
        elif kind == 'drive_distance':
            value = body['value']
            if type(value) not in (int, float) or not .05 <= value <= 2.:
                raise ValueError('Drive distance must be 0.05–2 metres forward')
            destination = {'x': pose['x'] + value * math.cos(pose['yaw']),
                           'y': pose['y'] + value * math.sin(pose['yaw']), 'yaw': pose['yaw']}
            clearance = grid.preview(pose, [destination], self.footprint(telemetry))
            if clearance['verdict'] != 'clear':
                raise PermissionError(f"Direct drive is {clearance['verdict']} at {clearance['first_issue']}")
        elif kind == 'turn_angle':
            value = body['value']
            if type(value) not in (int, float) or not 5 <= abs(value) <= 360:
                raise ValueError('Turn angle must be 5–360 degrees')
            # Preview a full sweep in small yaw increments, including turns over 180°.
            steps = math.ceil(abs(value) / 15)
            sweep = [{**pose, 'yaw': pose['yaw'] + math.radians(value) * i / steps}
                     for i in range(1, steps + 1)]
            clearance = grid.preview(pose, sweep, self.footprint(telemetry))
            if clearance['verdict'] != 'clear':
                raise PermissionError(f"Turn is {clearance['verdict']} at {clearance['first_issue']}")
        self.rpc('/metric/navigation/begin', data)
        self.wheel_control.reset()
        self.goal_seq = 0
        self.result = None
        self.goal = {**data, 'kind': 'direct' if kind in ('drive_distance', 'turn_angle') else kind,
                     'operation': kind, 'value': math.radians(value) if kind == 'turn_angle' else value if kind == 'drive_distance' else None,
                     'odom_start': self.odom.copy(), 'yaw_start': self.total_yaw,
                     'map_start_yaw': pose['yaw'], 'last_map_yaw': pose['yaw'],
                     'map_turn_total': 0.,
                     'started': time.monotonic()}
        if kind == 'navigate_to':
            goal = NavigateToPose.Goal()
            goal.pose = self.pose_message(body['target'])
            self.send_action(self.nav_client, goal, goal_id)
        elif kind == 'follow_route':
            goal = FollowPath.Goal()
            goal.path = NavPath()
            goal.path.header.frame_id = 'map'
            goal.path.header.stamp = self.get_clock().now().to_msg()
            goal.path.poses = [self.pose_message(p) for p in [pose, *body['waypoints']]]
            self.send_action(self.path_client, goal, goal_id)
        return {'goal_id': goal_id, 'state': 'running', 'map_version': grid.version}

    def footprint(self, telemetry):
        p = telemetry['profile']
        return {'length_m': p['length_m'], 'width_m': p['width_m'],
                'margin_m': p['margin_m'] + p['fallback_stop_m']}

    def pose_message(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(pose['x']), float(pose['y'])
        q = quaternion(float(pose['yaw']))
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        return msg

    def send_action(self, client, goal, goal_id):
        if not client.wait_for_server(timeout_sec=2.):
            self.finish('blocked')
            raise PermissionError('Nav2 action server unavailable')
        client.send_goal_async(goal).add_done_callback(lambda future: self.goal_accepted(goal_id, future))

    def goal_accepted(self, goal_id, future):
        try:
            handle = future.result()
            if not handle.accepted:
                with self.lock:
                    current = self.goal and self.goal['goal_id'] == goal_id
                if current:
                    self.finish('blocked', goal_id)
                return
            with self.lock:
                if not self.goal or self.goal['goal_id'] != goal_id:
                    handle.cancel_goal_async()
                    return
                self.action_handle = handle
            handle.get_result_async().add_done_callback(lambda result: self.goal_completed(goal_id, result))
        except Exception:
            with self.lock:
                current = self.goal and self.goal['goal_id'] == goal_id
            if current:
                self.finish('tracking_lost', goal_id)

    def goal_completed(self, goal_id, future):
        with self.lock:
            if not self.goal or self.goal['goal_id'] != goal_id:
                return
        try:
            result = future.result()
            # ROS action status 4 means succeeded. No timeout is arrival.
            self.finish('reached' if result.status == 4 else 'blocked', goal_id)
        except Exception:
            self.finish('tracking_lost', goal_id)

    def finish(self, outcome, expected_goal_id=None):
        with self.lock:
            if expected_goal_id and (not self.goal or self.goal['goal_id'] != expected_goal_id):
                return
            goal, self.goal = self.goal, None
            handle, self.action_handle = self.action_handle, None
            self.command = (0., 0.)
            self.wheel_control.reset()
            self.result = {'outcome': outcome, 'goal_id': goal['goal_id'] if goal else None,
                           'finished': time.time()}
        if not goal:
            return
        if handle:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        data = {k: goal[k] for k in ('mission_id', 'session', 'configuration_id', 'goal_id')}
        try:
            self.rpc('/metric/navigation/finish', {**data, 'outcome': outcome})
        except Exception:
            try: self.rpc('/metric/navigation/stop', {})
            except Exception: pass

    def save_map(self, mission_id):
        if not isinstance(mission_id, str) or not re.fullmatch(r'[0-9a-f]{24}', mission_id):
            raise ValueError('Invalid mission ID')
        with self.lock:
            if not self.map:
                raise PermissionError('No occupancy grid to save')
            snapshot = self.map.copy()
        directory = Path('/var/lib/ugv-maps') / mission_id
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'occupancy.json').write_text(json.dumps(snapshot, allow_nan=False))
        if not self.serialize_client.wait_for_service(timeout_sec=2.):
            raise PermissionError('SLAM Toolbox pose-graph service unavailable')
        req = SerializePoseGraph.Request()
        req.filename = str(directory / 'survey')
        done = threading.Event()
        future = self.serialize_client.call_async(req)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(20):
            raise PermissionError('Pose-graph serialization timed out')
        future.result()
        posegraph = directory / 'survey.posegraph'
        data = directory / 'survey.data'
        if not posegraph.is_file() or not data.is_file():
            raise PermissionError('Pose-graph files were not both written')
        return {'map_version': snapshot['map_version'], 'occupancy': str(directory / 'occupancy.json'),
                'posegraph': str(posegraph), 'scan_data': str(data)}


def create_app(node):
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = 65536

    @app.before_request
    def authenticated():
        if request.remote_addr not in ('127.0.0.1', '::1') or not secrets.compare_digest(
                request.headers.get('X-Robot-Key', ''), node.key):
            return jsonify(error='Navigation credential required'), 403

    @app.errorhandler(ValueError)
    def invalid(error): return jsonify(error=str(error)), 400

    @app.errorhandler(PermissionError)
    def blocked(error): return jsonify(error=str(error)), 423

    @app.get('/state')
    def state():
        with node.lock:
            return jsonify(map_version=node.map_version, pose=node.pose(),
                           calibration_status=('observation-only' if node.observation_profile else
                                               'measured-profile' if (node.telemetry or {}).get('profile') else 'sensor-only'),
                           scan_age_s=time.monotonic()-node.last_scan_at,
                           mission=(node.telemetry or {}).get('mission'),
                           goal=node.goal, result=node.result)

    @app.get('/map')
    def measured_map():
        with node.lock:
            if not node.map: raise PermissionError('No measured occupancy grid')
            return jsonify(node.map)

    @app.get('/scan')
    def scan():
        with node.lock:
            msg = node.scan
            if not msg: raise PermissionError('No LiDAR scan')
            return jsonify(sensor='LD06', frame_id=msg.header.frame_id,
                           stamp=stamp_seconds(msg.header.stamp), sequence=node.scan_seq,
                           angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                           range_min_m=msg.range_min, range_max_m=msg.range_max,
                           ranges_m=[v if math.isfinite(v) else None for v in msg.ranges],
                           intensities=list(msg.intensities))

    @app.get('/depth')
    def depth():
        with node.lock:
            if not node.depth: raise PermissionError('No VL53L5CX range grid')
            return jsonify(node.depth)

    @app.post('/navigate_to')
    def navigate(): return jsonify(node.begin('navigate_to', request.get_json()))

    @app.post('/follow_route')
    def follow(): return jsonify(node.begin('follow_route', request.get_json()))

    @app.post('/drive_distance')
    def drive(): return jsonify(node.begin('drive_distance', request.get_json()))

    @app.post('/turn_angle')
    def turn(): return jsonify(node.begin('turn_angle', request.get_json()))

    @app.post('/stop')
    def stop():
        node.finish('cancelled')
        return jsonify(ok=True)

    @app.post('/save_map')
    def save_map():
        return jsonify(node.save_map(request.get_json()['mission_id']))

    return app


def main():
    rclpy.init()
    node = Bridge()
    server = make_server('127.0.0.1', 5003, create_app(node), threaded=True)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    finally:
        node.finish('cancelled')
        server.shutdown()
        thread.join(1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
