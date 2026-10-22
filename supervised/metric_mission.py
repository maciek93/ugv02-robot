"""Pi-owned authority for continuous LiDAR navigation, independent of ROS."""
import copy
import hashlib
import json
import math
import secrets


def number(value, low, high, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be finite in [{low}, {high}]')
    return float(value)


def profile_hash(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True, allow_nan=False).encode()).hexdigest()


def validate_metric_profile(p):
    if not isinstance(p, dict) or p.get('version') != 2 or p.get('kind') != 'lidar-mapping':
        raise ValueError('A version 2 lidar-mapping profile is required')
    for key in ('configuration_id', 'lidar_serial', 'surface', 'evidence'):
        if not isinstance(p.get(key), str) or not p[key].strip():
            raise ValueError(f'Missing {key}')
    if len(p['evidence']) < 30:
        raise ValueError('Measured motion and mounting evidence required')
    for key, limits in {
        'length_m': (.1, 1.), 'width_m': (.1, 1.), 'track_m': (.1, 1.),
        'lidar_x_m': (-.5, .5), 'lidar_y_m': (-.5, .5), 'lidar_yaw_rad': (-math.pi, math.pi),
        'encoder_m_per_unit': (.00001, 1.), 'upper_speed_m_s': (.02, .5),
        'upper_turn_rad_s': (.05, 6.3), 'fallback_stop_m': (.05, 2.),
        'margin_m': (0, .5), 'forward_pwm': (1, 80), 'turn_pwm': (1, 80),
        'forward_m_s_at_pwm': (.02, .5), 'turn_rad_s_at_pwm': (.05, 6.3),
        'min_smooth_forward_pwm': (1, 80), 'min_smooth_turn_pwm': (1, 80),
    }.items():
        number(p.get(key), *limits, key)
    if p['fallback_stop_m'] < 3 * p['upper_speed_m_s']:
        raise ValueError('Fallback distance must include three-second ESP32 timeout')
    if p['min_smooth_forward_pwm'] > p['forward_pwm'] or p['min_smooth_turn_pwm'] > p['turn_pwm']:
        raise ValueError('Minimum smooth PWM exceeds calibrated PWM')
    signs = p.get('encoder_signs')
    if not isinstance(signs, list) or len(signs) != 2 or any(type(v) is not int or v not in (-1, 1) for v in signs):
        raise ValueError('Two verified encoder signs are required')
    polarity = p.get('motor_polarity')
    if not isinstance(polarity, list) or len(polarity) != 2 or any(type(v) is not int or v not in (-1, 1) for v in polarity):
        raise ValueError('Two motor polarity signs are required')
    return p


class MetricMission:
    def __init__(self, now):
        self.now = now
        self.proposal = self.active = self.goal = self.last = None
        self.used_goals = set()
        self.last_counts = self.last_sample_at = None

    def cancel(self, reason):
        if self.active or self.proposal:
            self.last = {**copy.deepcopy(self.active or self.proposal), 'outcome': reason}
        self.proposal = self.active = self.goal = None
        self.used_goals.clear()
        self.last_counts = self.last_sample_at = None

    def propose(self, spec, session, profile):
        validate_metric_profile(profile)
        if self.active:
            raise PermissionError('Finish the active mission first')
        required = {'configuration_id', 'rooms', 'doorways', 'start_room', 'duration_s', 'travel_m'}
        if not isinstance(spec, dict) or set(spec) != required:
            raise ValueError('Unexpected metric mission fields')
        if spec['configuration_id'] != profile['configuration_id']:
            raise PermissionError('Configuration changed')
        if not isinstance(spec['rooms'], list) or not 1 <= len(spec['rooms']) <= 20 or any(not isinstance(x, str) or not x.strip() for x in spec['rooms']):
            raise ValueError('Choose 1–20 named rooms')
        if spec['start_room'] not in spec['rooms']:
            raise ValueError('Starting room outside scope')
        if not isinstance(spec['doorways'], list):
            raise ValueError('Doorways must be a list')
        for d in spec['doorways']:
            if not isinstance(d, dict) or set(d) != {'id', 'from_room', 'to_room'} or any(d.get(k) not in spec['rooms'] for k in ('from_room', 'to_room')):
                raise ValueError('Invalid directed doorway')
        number(spec['duration_s'], 1, 600, 'duration_s')
        number(spec['travel_m'], .01, 10, 'travel_m')
        self.proposal = {**copy.deepcopy(spec), 'id': secrets.token_hex(12), 'session': session,
                         'profile_hash': profile_hash(profile), 'created': self.now()}
        return copy.deepcopy(self.proposal)

    def approve(self, mission_id, session, profile):
        p = self.proposal
        if not p or p['id'] != mission_id or p['session'] != session or self.now() - p['created'] > 120:
            raise PermissionError('Fresh matching mission proposal required')
        if profile_hash(validate_metric_profile(profile)) != p['profile_hash']:
            raise PermissionError('Profile changed')
        self.active = {**p, 'started': self.now(), 'expires': self.now() + p['duration_s'],
                       'traveled_m': 0., 'reserved_m': 0.}
        self.proposal = None
        self.used_goals.clear()
        self.last_counts = self.last_sample_at = None
        return copy.deepcopy(self.active)

    def observe(self, chassis, profile):
        """Count actual wheel travel once per UART sample; ignore repeated HTTP reads."""
        if not self.active:
            return
        data = chassis.get('data') or {}
        age = chassis.get('age_s')
        if (type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= .75 or
            any(type(data.get(k)) not in (int, float) or not math.isfinite(data[k])
                for k in ('odl', 'odr'))):
            raise PermissionError('Wheel travel telemetry unavailable')
        sample_at = self.now() - age
        counts = (float(data['odl']), float(data['odr']))
        if not self.goal:
            self.last_counts, self.last_sample_at = counts, sample_at
            return
        if self.last_sample_at is None:
            self.last_counts, self.last_sample_at = counts, sample_at
            return
        dt = sample_at - self.last_sample_at
        if dt < .03:
            return
        delta = max(abs(counts[i] - self.last_counts[i]) for i in (0, 1)) * profile['encoder_m_per_unit']
        self.last_counts, self.last_sample_at = counts, sample_at
        if dt > .75 or delta > profile['upper_speed_m_s'] * dt + .04:
            raise PermissionError('Encoder reset or implausible wheel travel')
        self.active['traveled_m'] += delta

    def check(self, identity, session, profile):
        m = self.active
        if not m or any(identity.get(k) != m[v] for k, v in (('mission_id', 'id'), ('session', 'session'), ('configuration_id', 'configuration_id'))):
            raise PermissionError('No matching active mission')
        if profile_hash(validate_metric_profile(profile)) != m['profile_hash']:
            raise PermissionError('Profile changed')
        if self.now() >= m['expires'] or m['reserved_m'] >= m['travel_m']:
            raise PermissionError('budget_exhausted')

    def begin(self, data, session, profile):
        self.check(data, session, profile)
        if self.goal:
            raise PermissionError('A metric goal is already active')
        goal_id = data.get('goal_id')
        if not isinstance(goal_id, str) or not goal_id or goal_id in self.used_goals:
            raise PermissionError('Duplicate or invalid goal')
        if data.get('action') not in ('navigate_to', 'follow_route', 'drive_distance', 'turn_angle'):
            raise ValueError('Unsupported metric goal')
        if self.active['traveled_m'] + profile['fallback_stop_m'] >= self.active['travel_m']:
            raise PermissionError('budget_exhausted')
        self.active['reserved_m'] = self.active['traveled_m'] + profile['fallback_stop_m']
        self.used_goals.add(goal_id)
        self.goal = {'id': goal_id, 'action': data['action'], 'started': self.now(),
                     'last_renew': None, 'seq': -1, 'reserved_until': self.now(),
                     'map_version': data.get('map_version')}
        return copy.deepcopy(self.goal)

    def renew(self, data, session, profile):
        self.check(data, session, profile)
        goal = self.goal
        now = self.now()
        if not goal or data.get('goal_id') != goal['id']:
            raise PermissionError('No matching metric goal')
        if goal['last_renew'] is not None and now - goal['last_renew'] > .1:
            raise PermissionError('Metric renewal missed 100 ms deadline')
        if type(data.get('seq')) is not int or data['seq'] != goal['seq'] + 1:
            raise PermissionError('Replayed or out-of-order renewal')
        left, right = (number(data.get(k), -80, 80, k) for k in ('left_pwm', 'right_pwm'))
        if type(left) is float and not left.is_integer() or type(right) is float and not right.is_integer():
            raise ValueError('PWM commands must be integers')
        deadline = min(now + .25, self.active['expires'])
        if self.last_sample_at is None:
            raise PermissionError('Wheel travel telemetry unavailable')
        # The measured distance is charged once, while only motion since the
        # latest sample and the ESP32 fallback remain reserved pessimistically.
        reserved = (self.active['traveled_m'] + profile['fallback_stop_m'] +
                    max(0., deadline - self.last_sample_at) * profile['upper_speed_m_s'])
        if reserved > self.active['travel_m']:
            raise PermissionError('budget_exhausted')
        self.active['reserved_m'] = reserved
        goal.update(last_renew=now, seq=data['seq'], reserved_until=deadline)
        return int(left), int(right), deadline

    def finish(self, goal_id, outcome):
        if not self.goal or self.goal['id'] != goal_id:
            raise PermissionError('No matching metric goal')
        if outcome not in ('reached', 'blocked', 'tracking_lost', 'cancelled', 'budget_exhausted'):
            raise ValueError('Invalid outcome')
        self.goal = None
        if self.active:
            self.active['reserved_m'] = self.active['traveled_m']
        if outcome in ('tracking_lost', 'cancelled', 'budget_exhausted'):
            self.cancel(outcome)

    def status(self):
        return copy.deepcopy({'proposal': self.proposal, 'active': self.active,
                              'goal': self.goal, 'last': self.last})
