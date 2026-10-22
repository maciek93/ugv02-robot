"""Pi-owned, one-use operator approval and finite movement quotas."""
import copy
import math
import secrets
import time


def finite(value, low, high, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be between {low} and {high}')
    return float(value)


class Policy:
    def __init__(self, now=time.monotonic):
        self.now = now
        self.session = None
        self.mode = None
        self.state = 'idle'
        self.proposal = None
        self.approval = None
        self.execution = None
        self.epoch = 0
        self.last_reason = 'startup'
        self.profile = None

    def invalidate(self, reason):
        self.epoch += 1
        self.proposal = self.approval = self.execution = None
        self.last_reason = reason

    def start(self, mode):
        if mode not in ('bench', 'calibration', 'tour', 'explore', 'manual', 'mapping'):
            raise ValueError('Choose bench, calibration, tour, explore, manual or mapping')
        if mode == 'tour' and not self.profile:
            raise PermissionError('Floor movement and obstacle calibration have not been validated')
        self.invalidate('session started')
        self.session = secrets.token_hex(16)
        self.mode = mode
        self.state = 'active'
        return self.session

    def propose(self, action, value, frame_id):
        if self.state != 'active' or self.execution:
            raise PermissionError('No active idle session')
        if action in ('continuous_forward', 'pwm_forward') and (self.mode == 'manual' or action == 'continuous_forward' and self.mode == 'calibration'):
            value = finite(value, .05, 30, 'continuous motor seconds')
            quota = value
        elif action in ('continuous_left', 'continuous_right') and self.mode == 'manual':
            value = finite(value, .05, 10, 'continuous turn seconds')
            quota = value
        elif action in ('pwm_left', 'pwm_right', 'pwm_spin_right', 'pwm_spin_left', 'pwm_pivot_left', 'pwm_pivot_right') and self.mode == 'manual':
            value = finite(value, .05, 30, 'PWM turn seconds')
            quota = value
        elif action == 'pulse_forward' and self.mode in ('bench', 'calibration'):
            # Explicitly approved floor sequences can total two seconds; each
            # lease still lasts at most 250 ms and requires a new observation.
            value = finite(value, .05, 1.0 if self.mode == 'bench' else 2.0, 'total test motor seconds')
            quota = value
        elif action == 'forward' and self.mode == 'tour':
            value = finite(value, 1, 50, 'distance cm')
            # Conservative upper travel speed measured on the floor; no reliance on setpoint.
            quota = value / 100 / self.profile['upper_speed_m_s']
        elif action in ('left', 'right') and self.mode == 'tour':
            value = finite(value, 1, 15, 'turn degrees')
            quota = value / self.profile['upper_turn_deg_s']
        elif action in ('pulse_left', 'pulse_right') and self.mode in ('bench', 'calibration'):
            value = finite(value, .05, 1.0 if self.mode == 'bench' else .25, 'total test motor seconds')
            quota = value
        else:
            raise ValueError('Action unavailable in this mode; no blind reversing')
        self.invalidate('new proposal')
        self.proposal = {'id': secrets.token_hex(16), 'session': self.session,
                         'epoch': self.epoch, 'action': action, 'value': value,
                         'motor_budget_s': quota, 'frame_id': frame_id,
                         'created': self.now()}
        return copy.deepcopy(self.proposal)

    def approve(self, proposal_id, clearance):
        if self.approval or self.execution:
            raise PermissionError('This proposal has already been approved or consumed')
        if not self.proposal or proposal_id != self.proposal['id'] or self.state != 'active':
            raise PermissionError('Proposal changed or session stopped')
        if self.now() - self.proposal['created'] > 120:
            raise PermissionError('Proposal expired; observe and propose again')
        if clearance is not True:
            raise PermissionError('Operator must confirm support or floor/turn clearance')
        self.approval = {'id': proposal_id, 'expires': self.now() + 30, 'epoch': self.epoch}

    def execute(self, proposal_id):
        if self.execution:
            raise PermissionError('This proposal is already executing')
        if (not self.approval or self.approval['id'] != proposal_id
                or self.approval['epoch'] != self.epoch
                or self.now() >= self.approval['expires'] or self.state != 'active'):
            raise PermissionError('Fresh single-use human approval required')
        self.approval = None
        self.execution = {**copy.deepcopy(self.proposal), 'started': self.now(),
                          'spent_s': 0., 'last_frame': None, 'odometry_cm': 0.}
        return copy.deepcopy(self.execution)

    def pulse(self, frame_id, odometry_delta=None):
        job = self.execution
        if not job or self.state != 'active':
            raise PermissionError('No approved execution')
        if job['action'].startswith('continuous_') or job['action'].startswith('pwm_'):
            raise PermissionError('Continuous trial requires renewed leases')
        if self.now() - job['started'] > 120:
            self.invalidate('leg time budget exceeded')
            raise PermissionError(self.last_reason)
        if frame_id == job['last_frame']:
            raise PermissionError('A fresh observation is required between pulses')
        if odometry_delta is not None:
            job['odometry_cm'] = odometry_delta
        remaining = job['motor_budget_s'] - job['spent_s']
        if remaining < .05 or (job['action'] == 'forward' and job['odometry_cm'] >= job['value']):
            self.invalidate('approved leg completed')
            return None
        duration = min(.25, remaining)
        job['spent_s'] += duration
        job['last_frame'] = frame_id
        action = job['action']
        left, right = .05, .05
        if 'left' in action:
            left = -.05
        if 'right' in action:
            right = -.05
        return left, right, duration

    def status(self):
        return {'session': self.session, 'mode': self.mode, 'state': self.state,
                'epoch': self.epoch, 'last_reason': self.last_reason,
                'proposal': copy.deepcopy(self.proposal),
                'approved': bool(self.approval and self.now() < self.approval['expires']),
                'execution': copy.deepcopy(self.execution), 'calibrated': bool(self.profile)}
