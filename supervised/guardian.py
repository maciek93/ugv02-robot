"""Minimal Pi service: motor watchdog here, camera in a separate process."""
import base64
import functools
import hmac
import json
import logging
import os
from pathlib import Path
import signal
import threading
import time

from flask import Flask, jsonify, request
from werkzeug.serving import make_server
from camera import Camera, atomic_json
from policy import Policy, finite
from transport import Chassis
from mission import Mission
from explore_profile import validate_profile, camera_matches
from metric_mission import MetricMission, validate_metric_profile


class Guardian:
    def __init__(self, chassis, camera, directory, now=time.monotonic, wall_now=time.time):
        self.chassis, self.camera, self.now, self.wall_now = chassis, camera, now, wall_now
        self.directory = Path(directory)
        self.policy = Policy(now)
        self.mission = Mission(now)
        self.metric_mission = MetricMission(now)
        self.explore_profile = None
        explore_path = self.directory / "explore-profile.json"
        if explore_path.exists():
            self.explore_profile = json.loads(explore_path.read_text())
        metric_path = self.directory / 'metric-profile.json'
        self.metric_profile = json.loads(metric_path.read_text()) if metric_path.exists() else None
        self.metric_scan = None
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.heartbeat_at = 0
        self.vision = None
        self.accepted_frames = {}
        self.deadline = 0
        self.continuous_end = 0
        self.direction = (0, 0)
        self.last_motor_write_at = None
        self.last_pwm_command = None
        self.retry_stop = False
        self.last_pulse_end = 0
        self.baseline_odometry = None
        self.proposal_signature = None
        self.sensor_providers = []  # Future ranging providers return rejection reasons.
        self.thread = threading.Thread(target=self.watch, daemon=True, name='motor-watchdog')
        profile = self.directory / 'profile.json'
        if profile.exists():
            self.policy.profile = json.loads(profile.read_text())
        self.thread.start()

    def stop(self, reason, invalidate=True):
        self.deadline = 0
        self.continuous_end = 0
        self.direction = (0, 0)
        self.last_motor_write_at = None
        self.last_pwm_command = None
        self.last_pulse_end = self.now()
        if invalidate:
            self.policy.invalidate(reason)
            self.mission.cancel(reason)
            self.metric_mission.cancel(reason)
        try:
            if getattr(self.chassis, 'motor_mode', None) == 'pwm':
                self.chassis.pwm(0, 0)
            else:
                self.chassis.wheels(0, 0)
            self.retry_stop = False
        except Exception:
            self.retry_stop = True
            raise

    def unsafe(self, moving=False):
        if self.policy.state != 'active':
            return 'Session is paused or ended'
        if self.policy.mode == 'manual':
            return None
        if self.now() - self.heartbeat_at > .6:
            return 'Operator connection lost'
        telemetry = self.chassis.status()
        if telemetry['error'] or telemetry['age_s'] > .75:
            return 'Chassis telemetry stale or failed'
        if self.policy.mode == 'mapping':
            p = self.metric_profile
            if not p:
                return 'Measured LiDAR/motion profile is not installed'
            if tuple(p['motor_polarity']) != tuple(getattr(self.chassis, 'motor_polarity', ())):
                return 'Motor configuration changed'
            scan = self.metric_scan
            if not scan or self.now() - scan['received'] > .5 or scan['valid_beams'] < 50:
                return 'LiDAR scan missing or stale'
            m = self.metric_mission.active
            if m and (self.now() >= m['expires'] or m['reserved_m'] >= m['travel_m']):
                return 'budget_exhausted'
            goal = self.metric_mission.goal
            if goal and goal['last_renew'] is not None and self.now() - goal['last_renew'] > .1:
                return 'Metric navigation renewal missed 100 ms deadline'
            return None
        frame = self.camera.read()
        if frame.get('error') or frame.get('quality_reason'):
            return frame.get('error') or frame['quality_reason']
        vision = self.vision
        if (not vision or self.now() - vision['received'] > .5
                or self.now() - vision['captured'] > .5):
            return 'Vision observation missing or stale'
        if vision.get('error'):
            return 'Mac perception failed'
        if vision.get('hazard') and self.policy.mode != 'bench':
            return vision['hazard']
        if self.policy.mode == 'explore':
            p = self.explore_profile
            if not p or not camera_matches(p, frame):
                return 'Camera configuration or focus changed'
            if tuple(p['motor_polarity']) != tuple(getattr(self.chassis, 'motor_polarity', ())):
                return 'Motor configuration changed'
            geometry = vision.get('geometry', {})
            if geometry.get('configuration_id') != p['configuration_id']:
                return 'Geometry configuration changed'
            if geometry.get('error') or geometry.get('tracking') != 'good':
                return geometry.get('error') or 'Visual tracking unavailable'
            if frame.get('sharpness', 0) < p['min_sharpness']:
                return 'Image blurred'
            if frame.get('identical_frames', 0) >= 3:
                return 'Repeated camera frames'
            score = vision.get('near_score')
            if not vision.get('depth_ready') or type(score) not in (int,float) or not p['score_low'] <= score < p['near_limit']:
                return 'Relative-depth veto or unvalidated score'
            if self.mission.active:
                if self.now() >= self.mission.active['expires'] or self.mission.active['reserved_m'] >= self.mission.active['travel_m']:
                    return 'budget_exhausted'
            leg = self.mission.leg
            if leg and leg['last_renew'] is not None and self.now()-leg['last_renew'] > .1:
                return 'Navigation renewal missed 100 ms deadline'
        if self.policy.mode == 'tour':
            profile = self.policy.profile
            if not profile or not vision.get('depth_ready'):
                return 'Depth safeguard not validated'
            score = vision.get('near_score')
            if type(score) not in (int, float) or not profile['score_low'] <= score <= profile['score_high']:
                return 'Depth outside validated range'
            if score >= profile['near_limit']:
                return 'Obstacle cue in travel corridor'
        for provider in self.sensor_providers:
            reason = provider(*self.direction)
            if reason:
                return reason
        if moving and self.policy.mode == 'tour':
            data = telemetry['data']
            if self.baseline_odometry is None or not self.valid_odometry(data):
                return 'Wheel travel estimate unavailable'
            travel = max(abs(data[k] - baseline) for k, baseline in zip(('odl','odr'), self.baseline_odometry))
            if travel >= 3 and frame.get('identical_frames', 0) >= 5:
                return 'Camera unchanged despite wheel travel; possible frozen capture or stalled robot'
        return None

    @staticmethod
    def valid_odometry(data):
        return bool(data and all(type(data.get(k)) in (int, float) and
                    __import__('math').isfinite(data[k]) for k in ('odl', 'odr')))

    def watch(self):
        while not self.quit.wait(.02):
            with self.lock:
                try:
                    if self.retry_stop:
                        self.stop('retry failed Stop')
                    if self.deadline:
                        reason = self.unsafe(moving=True)
                        if reason:
                            self.stop(reason)
                        elif self.now() >= self.deadline:
                            reason = ('continuous trial completed' if self.now() >= self.continuous_end else 'continuous lease expired') if self.continuous_end else 'pulse ended'
                            self.stop(reason,
                                      invalidate=bool(self.continuous_end or self.mission.leg or self.metric_mission.goal))
                    elif self.policy.approval or self.mission.active or self.metric_mission.active:
                        reason = self.unsafe()
                        if reason:self.stop(reason)
                    if (self.policy.state == 'active' and self.policy.mode != 'manual'
                            and self.now() - self.heartbeat_at > 2):
                        self.stop('Operator disconnected; session paused')
                        self.policy.state = 'paused'
                        # Stop first; do not hold motor supervision waiting for camera shutdown.
                        threading.Thread(target=self.camera.close, daemon=True).start()
                except Exception:
                    logging.exception('Watchdog Stop failed; will retry')

    def state(self):
        frame = self.camera.read() if self.policy.state == 'active' else {'error': 'Camera is off'}
        return {**self.policy.status(), 'mission': self.mission.status(), 'metric_mission': self.metric_mission.status(),
                'metric_profile': self.metric_profile, 'metric_scan': self.metric_scan,
                'last_stop_captured': self.last_pulse_end, 'explore_profile': self.explore_profile, 'chassis': self.chassis.status(), 'camera': frame,
                'vision': self.vision, 'moving': bool(self.deadline),
                'pulse_remaining_s': max(0, self.deadline - self.now()),
                'blocking_reason': self.unsafe() if self.policy.state == 'active' else None,
                'max_requested_speed_m_s': .10 if self.policy.mode == 'manual' else self.metric_profile['upper_speed_m_s'] if self.policy.mode == 'mapping' and self.metric_profile else .05,
                'max_pulse_s': .25}

    def close(self):
        with self.lock:
            try:
                self.stop('guardian shutdown')
            finally:
                self.policy.state = 'ended'
                self.quit.set()
        self.thread.join(.5)
        self.camera.close()
        self.chassis.close()


def create_app(guardian, keys):
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = 256 * 1024

    def auth(role='agent'):
        def decorate(function):
            @functools.wraps(function)
            def wrapped(*args, **kwargs):
                supplied = request.headers.get('X-Robot-Key', '')
                accepted = ([keys['operator']] if role == 'operator' else
                            [keys['vision']] if role == 'vision' else
                            [keys.get('navigation', '')] if role == 'navigation' else
                            list(keys.values()) if role == 'reader' else
                            [keys['agent'], keys['operator']])
                if request.remote_addr not in ('127.0.0.1', '::1') or not any(
                        bool(key) and hmac.compare_digest(supplied, key) for key in accepted):
                    return jsonify(error='Authenticated SSH-local access required'), 403
                with guardian.lock:
                    return function(*args, **kwargs)
            return wrapped
        return decorate

    @app.errorhandler(ValueError)
    def invalid(error):
        return jsonify(error=str(error)), 400

    @app.errorhandler(PermissionError)
    def blocked(error):
        return jsonify(error=str(error)), 423

    def body(*fields):
        data = request.get_json(silent=True)
        if not isinstance(data, dict) or set(data) != set(fields):
            raise ValueError('Unexpected request fields')
        return data

    @app.get('/health')
    @auth('reader')
    def health():
        return jsonify(guardian.state())

    @app.get('/metric/telemetry')
    @auth('navigation')
    def metric_telemetry():
        return jsonify(session=guardian.policy.session, mode=guardian.policy.mode,
                       state=guardian.policy.state, mission=guardian.metric_mission.status(),
                       profile=guardian.metric_profile, scan=guardian.metric_scan,
                       chassis=guardian.chassis.status(), moving=bool(guardian.deadline),
                       last_reason=guardian.policy.last_reason)

    @app.post('/operator/session')
    @auth('operator')
    def session():
        data = body('operation', 'mode')
        operation = data['operation']
        guardian.stop('session ' + operation)
        if operation in ('start', 'resume'):
            if guardian.policy.state == 'active':
                raise PermissionError('Pause or end the existing session first')
            if data['mode'] == 'explore':
                validate_profile(guardian.explore_profile)
            if data['mode'] == 'mapping':
                validate_metric_profile(guardian.metric_profile)
            guardian.policy.start(data['mode'])
            guardian.heartbeat_at = guardian.now()
            guardian.vision = None
            if data['mode'] == 'mapping':
                guardian.metric_scan = None
            guardian.camera.start()
        elif operation in ('pause', 'end'):
            guardian.policy.state = 'paused' if operation == 'pause' else 'ended'
            guardian.camera.close()
            guardian.vision = None
        else:
            raise ValueError('Unknown session operation')
        return jsonify(guardian.state())

    @app.post('/operator/heartbeat')
    @auth('operator')
    def heartbeat():
        data = body('session')
        if data['session'] != guardian.policy.session:
            raise PermissionError('Old session')
        guardian.heartbeat_at = guardian.now()
        return jsonify(ok=True, mission_id=guardian.mission.active['id'] if guardian.mission.active else None,
                       metric_mission_id=guardian.metric_mission.active['id'] if guardian.metric_mission.active else None,
                       last_reason=guardian.policy.last_reason)

    @app.get('/observe')
    @auth('reader')
    def observe():
        frame = guardian.camera.read()
        if frame.get('error'):
            raise PermissionError(frame['error'])
        try:
            jpeg = (guardian.camera.directory / (frame['frame_id'] + '.jpg')).read_bytes()
        except OSError:
            raise PermissionError('Frame expired; observe again')
        return jsonify(**frame, session=guardian.policy.session,
                       jpeg=base64.b64encode(jpeg).decode(), chassis=guardian.chassis.status(),
                       mission=guardian.mission.active)

    @app.post('/perception')
    @auth('vision')
    def perception():
        fields = ['frame_id', 'captured', 'session', 'depth_ready', 'near_score', 'hazard', 'error', 'signature']
        if guardian.policy.mode == 'explore': fields.append('geometry')
        data = body(*fields)
        if 'geometry' in data:
            g = data['geometry']
            if not isinstance(g, dict) or set(g) != {'configuration_id','tracking','error'}:
                raise ValueError('Bad geometry summary')
            if g['tracking'] not in ('good','lost') or (g['error'] is not None and not isinstance(g['error'],str)):
                raise ValueError('Bad tracking result')
        if data['session'] != guardian.policy.session or guardian.policy.state != 'active':
            raise PermissionError('Observation belongs to an old session')
        captured = finite(data['captured'], guardian.now() - .5, guardian.now(), 'frame capture time')
        if not isinstance(data['frame_id'], str) or '/' in data['frame_id']:
            raise ValueError('Bad frame ID')
        if not (guardian.camera.directory / (data['frame_id'] + '.jpg')).exists():
            raise PermissionError('Unknown camera frame')
        original = json.loads((guardian.camera.directory / (data['frame_id'] + '.json')).read_text())
        if captured != original['captured']:
            raise PermissionError('Capture timestamp does not match camera frame')
        if guardian.vision and captured <= guardian.vision['captured']:
            raise PermissionError('Replayed observation')
        if type(data['depth_ready']) is not bool:
            raise ValueError('Bad depth readiness')
        if data['near_score'] is not None:
            finite(data['near_score'], -1e6, 1e6, 'near score')
        if not isinstance(data['signature'], list) or len(data['signature']) != 48:
            raise ValueError('Bad scene signature')
        for number in data['signature']:
            finite(number, 0, 255, 'signature pixel')
        if any(data[k] is not None and (not isinstance(data[k], str) or len(data[k]) > 200)
               for k in ('error', 'hazard')):
            raise ValueError('Bad perception result')
        guardian.vision = {**data, 'captured': captured, 'received': guardian.now()}
        guardian.accepted_frames[data['frame_id']] = guardian.vision
        guardian.accepted_frames = {k:v for k,v in guardian.accepted_frames.items() if guardian.now()-v['captured']<=.5}
        if data['error'] or (data['hazard'] and guardian.policy.mode not in ('bench', 'manual')):
            guardian.stop(data['error'] or data['hazard'])
        return jsonify(ok=True)

    @app.post('/propose')
    @auth()
    def propose():
        data = body('action', 'value')
        reason = guardian.unsafe()
        if reason:
            raise PermissionError(reason)
        proposal = guardian.policy.propose(data['action'], data['value'],
                                           guardian.vision.get('frame_id') if guardian.vision else None)
        guardian.proposal_signature = guardian.vision.get('signature') if guardian.vision else None
        return jsonify(proposal)

    @app.post('/operator/approve')
    @auth('operator')
    def approve():
        data = body('proposal_id', 'clearance')
        reason = guardian.unsafe()
        if reason:
            raise PermissionError(reason)
        guardian.policy.approve(data['proposal_id'], data['clearance'])
        return jsonify(ok=True)

    @app.post('/execute')
    @auth()
    def execute():
        data = body('proposal_id')
        reason = guardian.unsafe()
        if reason:
            guardian.policy.invalidate(reason)
            raise PermissionError(reason)
        if guardian.policy.mode != 'manual':
            change = sum(abs(a-b) for a, b in zip(guardian.proposal_signature or [], guardian.vision['signature'])) / 48
            if change > 15:
                guardian.policy.invalidate('Scene changed since proposal')
                raise PermissionError('Scene changed; new proposal and approval required')
        job = guardian.policy.execute(data['proposal_id'])
        telemetry = guardian.chassis.status()['data']
        guardian.baseline_odometry = (telemetry['odl'], telemetry['odr']) if guardian.valid_odometry(telemetry) else None
        return jsonify(job)

    @app.post('/pulse')
    @auth()
    def pulse():
        data = body('proposal_id')
        if guardian.deadline or guardian.now() - guardian.last_pulse_end < .1:
            raise PermissionError('Wait for Stop and settling before another pulse')
        if not guardian.policy.execution or data['proposal_id'] != guardian.policy.execution['id']:
            raise PermissionError('No matching approved execution')
        reason = guardian.unsafe(moving=True)
        if reason:
            guardian.stop(reason)
            raise PermissionError(reason)
        if guardian.vision['captured'] <= guardian.last_pulse_end:
            raise PermissionError('Observe after Stop before moving again')
        distance = None
        telemetry = guardian.chassis.status()['data']
        if guardian.policy.mode == 'tour':
            if not guardian.valid_odometry(telemetry) or guardian.baseline_odometry is None:
                guardian.stop('Travel estimate missing')
                raise PermissionError('Travel estimate missing')
            deltas = [abs(telemetry[k] - start) for k, start in zip(('odl', 'odr'), guardian.baseline_odometry)]
            distance = max(deltas)
            job = guardian.policy.execution
            if max(deltas) > guardian.policy.profile['upper_speed_m_s'] * 100 * (job['spent_s'] + .25) + 3:
                guardian.stop('Travel estimate inconsistent with calibration')
                raise PermissionError('Travel estimate inconsistent with calibration')
        command = guardian.policy.pulse(guardian.vision['frame_id'], distance)
        if command is None:
            guardian.stop('approved leg completed')
            return jsonify(done=True)
        left, right, duration = command
        guardian.direction = (left, right)
        guardian.deadline = guardian.now() + duration
        try:
            guardian.chassis.wheels(left, right)
            if guardian.now() >= guardian.deadline:
                guardian.stop('write exceeded lease')
        except Exception:
            guardian.stop('motor write failure')
            raise
        return jsonify(done=False, pulse_s=duration)

    @app.post('/stop')
    @auth()
    def stop():
        body()
        guardian.stop('explicit Stop')
        return jsonify(guardian.state())

    @app.post('/renew')
    @auth()
    def renew():
        data = body('proposal_id')
        job = guardian.policy.execution
        if not job or job['id'] != data['proposal_id'] or not (job['action'].startswith('continuous_') or job['action'].startswith('pwm_')):
            raise PermissionError('No matching approved continuous trial')
        reason = guardian.unsafe(moving=True)
        if reason:
            guardian.stop(reason)
            raise PermissionError(reason)
        now = guardian.now()
        if guardian.continuous_end and (now >= guardian.deadline or now >= guardian.continuous_end):
            guardian.stop('continuous trial completed' if now >= guardian.continuous_end else 'continuous lease expired')
            return jsonify(done=True)
        # Renewal may reuse the current, still-fresh observation. Camera and
        # perception freshness are checked above independently on every renewal
        # and by the watchdog; model cadence must not introduce stop/start gaps.
        if not guardian.continuous_end:
            if guardian.policy.mode != 'manual' and guardian.vision['captured'] <= guardian.last_pulse_end:
                raise PermissionError('Observe after Stop before moving again')
            guardian.continuous_end = now + job['motor_budget_s']
        job['last_frame'] = guardian.vision.get('frame_id') if guardian.vision else None
        job['spent_s'] = job['motor_budget_s'] - (guardian.continuous_end - now)
        speed = .10 if guardian.policy.mode == 'manual' else .05
        left, right = speed, speed
        if job['action'] in ('continuous_left', 'pwm_left', 'pwm_spin_left'):
            left, right = -speed, speed
        elif job['action'] in ('continuous_right', 'pwm_right', 'pwm_spin_right'):
            left, right = speed, -speed
        guardian.direction = (left, right)
        guardian.deadline = min(now + .25, guardian.continuous_end)
        try:
            # Renew the Pi lease on every call, but only refresh an identical
            # manual setpoint about once per second for the ESP32 heartbeat.
            if (guardian.policy.mode != 'manual' or guardian.last_motor_write_at is None
                    or now - guardian.last_motor_write_at >= 1.0):
                if job['action'].startswith('pwm_'):
                    pwm_turn = 80 if 'spin' in job['action'] else 50
                    if job['action'] in ('pwm_spin_right', 'pwm_right'):
                        guardian.chassis.pwm(pwm_turn, -pwm_turn)
                    elif job['action'] in ('pwm_spin_left', 'pwm_left'):
                        guardian.chassis.pwm(-pwm_turn, pwm_turn)
                    elif job['action'] == 'pwm_pivot_right':
                        guardian.chassis.pwm(55, 0)
                    elif job['action'] == 'pwm_pivot_left':
                        guardian.chassis.pwm(0, 55)
                    else:  # pwm_forward
                        guardian.chassis.pwm(50, 50)
                else:
                    guardian.chassis.wheels(left, right)
                guardian.last_motor_write_at = now
            if guardian.now() >= guardian.deadline:
                guardian.stop('write exceeded lease')
        except Exception:
            guardian.stop('motor write failure')
            raise
        return jsonify(done=False, lease_s=max(0, guardian.deadline - guardian.now()))

    @app.post('/operator/profile')
    @auth('operator')
    def profile():
        data = body('upper_speed_m_s', 'upper_turn_deg_s', 'near_limit', 'score_low', 'score_high', 'evidence')
        finite(data['upper_speed_m_s'], .05, .5, 'measured upper speed')
        finite(data['upper_turn_deg_s'], 1, 360, 'measured upper turn rate')
        for key in ('near_limit', 'score_low', 'score_high'):
            finite(data[key], -1e6, 1e6, key)
        if not data['score_low'] < data['near_limit'] < data['score_high']:
            raise ValueError('Depth thresholds must be ordered')
        if not isinstance(data['evidence'], str) or len(data['evidence']) < 30:
            raise ValueError('Measured calibration and obstacle-test evidence required')
        guardian.stop('calibration updated')
        guardian.policy.profile = data
        atomic_json(guardian.directory / 'profile.json', data)
        return jsonify(ok=True)

    @app.post('/operator/explore-profile')
    @auth('operator')
    def explore_profile():
        data = request.get_json()
        validate_profile(data)
        guardian.stop('exploration profile changed')
        guardian.explore_profile = data
        atomic_json(guardian.directory / 'explore-profile.json', data)
        return jsonify(ok=True)

    @app.post('/operator/metric-profile')
    @auth('operator')
    def metric_profile():
        data = request.get_json()
        validate_metric_profile(data)
        guardian.stop('metric profile changed')
        guardian.metric_profile = data
        atomic_json(guardian.directory / 'metric-profile.json', data)
        return jsonify(ok=True)

    @app.post('/metric/scan')
    @auth('navigation')
    def metric_scan():
        data = body('sequence', 'stamp', 'frame_id', 'valid_beams')
        if type(data['sequence']) is not int or data['sequence'] < 0:
            raise ValueError('Invalid scan sequence')
        if not isinstance(data['frame_id'], str) or data['frame_id'] != 'base_laser':
            raise ValueError('Unexpected LiDAR frame')
        finite(data['stamp'], 0, 1e12, 'scan timestamp')
        finite(data['valid_beams'], 0, 2000, 'valid beams')
        if abs(guardian.wall_now() - data['stamp']) > .5:
            raise PermissionError('LiDAR source timestamp is stale or ahead of Pi clock')
        prior = guardian.metric_scan
        if prior and (data['sequence'] <= prior['sequence'] or data['stamp'] <= prior['stamp']):
            raise PermissionError('Repeated or old LiDAR scan')
        guardian.metric_scan = {**data, 'received': guardian.now()}
        return jsonify(ok=True)

    @app.post('/metric/mission/propose')
    @auth()
    def metric_mission_propose():
        if guardian.policy.mode != 'mapping' or guardian.policy.state != 'active':
            raise PermissionError('Start a mapping session first')
        reason = guardian.unsafe()
        if reason: raise PermissionError(reason)
        return jsonify(guardian.metric_mission.propose(request.get_json(), guardian.policy.session, guardian.metric_profile))

    @app.post('/operator/metric/mission/approve')
    @auth('operator')
    def metric_mission_approve():
        data = body('mission_id')
        reason = guardian.unsafe()
        if reason: raise PermissionError(reason)
        return jsonify(guardian.metric_mission.approve(data['mission_id'], guardian.policy.session, guardian.metric_profile))

    def metric_navigation_context(data):
        if guardian.policy.mode != 'mapping':
            raise PermissionError('Mapping mode required')
        reason = guardian.unsafe()
        if reason:
            guardian.stop(reason)
            raise PermissionError(reason)
        guardian.metric_mission.observe(guardian.chassis.status(), guardian.metric_profile)
        guardian.metric_mission.check(data, guardian.policy.session, guardian.metric_profile)

    @app.post('/metric/navigation/begin')
    @auth('navigation')
    def metric_navigation_begin():
        data = body('mission_id', 'session', 'configuration_id', 'goal_id', 'action', 'map_version')
        metric_navigation_context(data)
        if guardian.deadline:
            raise PermissionError('Previous motor command is still active')
        return jsonify(guardian.metric_mission.begin(data, guardian.policy.session, guardian.metric_profile))

    @app.post('/metric/navigation/renew')
    @auth('navigation')
    def metric_navigation_renew():
        data = body('mission_id', 'session', 'configuration_id', 'goal_id', 'seq', 'left_pwm', 'right_pwm')
        try:
            metric_navigation_context(data)
            left, right, deadline = guardian.metric_mission.renew(data, guardian.policy.session, guardian.metric_profile)
            guardian.direction = (left, right)
            guardian.deadline = deadline
            now = guardian.now()
            if guardian.last_pwm_command != (left, right) or guardian.last_motor_write_at is None or now - guardian.last_motor_write_at >= 1.0:
                guardian.chassis.pwm(left, right)
                guardian.last_pwm_command = (left, right)
                guardian.last_motor_write_at = now
            if guardian.now() >= deadline:
                raise PermissionError('Motor write exceeded lease')
        except Exception:
            guardian.stop('metric navigation renewal failed')
            raise
        return jsonify(lease_s=max(0, deadline - guardian.now()))

    @app.post('/metric/navigation/finish')
    @auth('navigation')
    def metric_navigation_finish():
        data = body('mission_id', 'session', 'configuration_id', 'goal_id', 'outcome')
        metric_navigation_context(data)
        guardian.metric_mission.finish(data['goal_id'], data['outcome'])
        guardian.stop(data['outcome'], invalidate=data['outcome'] not in ('reached', 'blocked'))
        return jsonify(guardian.state())

    @app.post('/metric/navigation/stop')
    @auth('navigation')
    def metric_navigation_stop():
        guardian.stop('metric navigation cancelled')
        return jsonify(guardian.state())

    @app.post('/mission/propose')
    @auth()
    def mission_propose():
        if guardian.policy.mode != 'explore' or guardian.policy.state != 'active':
            raise PermissionError('Start an exploration observation session first')
        reason = guardian.unsafe()
        if reason: raise PermissionError(reason)
        return jsonify(guardian.mission.propose(request.get_json(), guardian.policy.session, guardian.explore_profile))

    @app.post('/operator/mission/approve')
    @auth('operator')
    def mission_approve():
        data = body('mission_id')
        reason = guardian.unsafe()
        if reason: raise PermissionError(reason)
        return jsonify(guardian.mission.approve(data['mission_id'], guardian.policy.session, guardian.explore_profile))

    def navigation_context(data):
        if guardian.policy.mode != 'explore': raise PermissionError('Exploration mode required')
        reason = guardian.unsafe()
        if reason:
            guardian.stop(reason)
            raise PermissionError(reason)
        source = guardian.accepted_frames.get(data.get('frame_id'))
        if not source or source['session'] != guardian.policy.session or guardian.now()-source['captured'] > .5 or source.get('geometry',{}).get('tracking') != 'good':
            raise PermissionError('Use fresh accepted geometry frame')

    @app.post('/navigation/probe')
    @auth('navigation')
    def navigation_probe():
        # Stationary end-to-end timing probe. No motor write or authority grant.
        data = body('session','frame_id','captured')
        if guardian.policy.mode != 'calibration' or guardian.deadline or guardian.policy.execution:
            raise PermissionError('Stationary calibration required')
        if data['session'] != guardian.policy.session or not isinstance(data['frame_id'],str) or '/' in data['frame_id']:
            raise PermissionError('Invalid probe frame')
        path=guardian.camera.directory/(data['frame_id']+'.json')
        if not path.is_file():raise PermissionError('Probe frame expired')
        original=json.loads(path.read_text())
        if original['captured'] != data['captured']:raise PermissionError('Probe timestamp mismatch')
        age=finite(guardian.now()-data['captured'],0,.5,'capture-to-probe seconds')
        return jsonify(capture_to_probe_s=age)

    @app.post('/navigation/begin')
    @auth('navigation')
    def navigation_begin():
        data = body('mission_id','session','configuration_id','goal_id','frame_id','action','value')
        navigation_context(data)
        if guardian.deadline or guardian.accepted_frames[data['frame_id']]['captured'] <= guardian.last_pulse_end:
            raise PermissionError('Fresh stationary observation required')
        return jsonify(guardian.mission.begin(data, guardian.policy.session, guardian.explore_profile))

    @app.post('/navigation/renew')
    @auth('navigation')
    def navigation_renew():
        data = body('mission_id','session','configuration_id','goal_id','frame_id','seq','left','right')
        try:
            navigation_context(data)
            left, right, deadline = guardian.mission.renew(data, guardian.policy.session, guardian.explore_profile)
            guardian.direction = (left, right)
            guardian.deadline = deadline
            guardian.chassis.wheels(left, right)
            if guardian.now() >= deadline: raise PermissionError('Motor write exceeded lease')
        except Exception:
            guardian.stop('navigation renewal failed')
            raise
        return jsonify(lease_s=max(0, deadline-guardian.now()))

    @app.post('/navigation/finish')
    @auth('navigation')
    def navigation_finish():
        data = body('mission_id','session','configuration_id','goal_id','outcome')
        guardian.mission.check(data, guardian.policy.session, guardian.explore_profile)
        guardian.mission.finish_leg(data['goal_id'], data['outcome'])
        guardian.stop(data['outcome'], invalidate=data['outcome'] != 'reached')
        return jsonify(guardian.state())

    @app.post('/navigation/stop')
    @auth('navigation')
    def navigation_stop():
        guardian.stop('navigation cancelled')
        return jsonify(guardian.state())

    @app.post('/api/drive')
    @app.post('/json')
    @app.post('/send_command')
    def legacy():
        return jsonify(error='Legacy driving disabled; propose and obtain human approval'), 403

    return app


def main():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    logging.getLogger('werkzeug').setLevel(logging.WARNING)
    directory = Path(os.environ.get('UGV_CONFIG', str(Path.home() / '.config/ugv-supervised')))
    keys = json.loads((directory / 'keys.json').read_text())
    camera_config = directory / 'camera.json'
    rotation = json.loads(camera_config.read_text()).get('rotation_degrees', 0) if camera_config.exists() else 0
    camera = Camera(os.environ.get('UGV_RUNTIME', '/run/ugv-supervised'),
                    '/dev/v4l/by-id/usb-USB_USB-video-index0', rotation=rotation)
    motor_config = directory / 'motors.json'
    polarity = json.loads(motor_config.read_text()).get('motor_polarity', [1, 1]) if motor_config.exists() else [1, 1]
    guardian = Guardian(Chassis(motor_polarity=polarity), camera, directory)
    server = make_server('127.0.0.1', 5000, create_app(guardian, keys), threaded=True)
    server.daemon_threads = True
    stopping = threading.Event()

    def stop(*_):
        stopping.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    server.timeout = .1
    try:
        while not stopping.is_set():
            server.handle_request()
    finally:
        guardian.close()
        server.server_close()
        logging.info('Guardian exited cleanly; camera released and Stop sent')


if __name__ == '__main__':
    main()
