"""Finite movement leases; every motor write passes this guard."""
import math
import threading
import time

class MotionGuard:
    MAX_SPEED = 0.10
    MAX_DURATION = 0.35

    def __init__(self, write, enabled, unsafe_reason, now=time.monotonic):
        self.write = write
        self.enabled = enabled
        self.unsafe_reason = unsafe_reason
        self.now = now
        self.lock = threading.RLock()
        self.owner = None
        self.deadline = 0.0
        self.active = False
        self.last_stop = 'startup'

    def drive(self, left, right, duration, owner):
        for name, value, limit in [('left', left, self.MAX_SPEED),
                                   ('right', right, self.MAX_SPEED)]:
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > limit:
                raise ValueError(f'{name} must be finite and within +/-{limit} m/s')
        if type(duration) not in (int, float) or not math.isfinite(duration) or not 0.05 <= duration <= self.MAX_DURATION:
            raise ValueError('duration must be between 0.05 and 0.35 seconds')
        if not isinstance(owner, str) or not owner or len(owner) > 80:
            raise ValueError('A short controller owner ID is required')
        with self.lock:
            self.tick()
            if left == right == 0:
                self.stop('explicit stop')
                return self.status()
            if not self.enabled():
                raise PermissionError('Motion is disarmed')
            reason = self.unsafe_reason(left, right)
            if reason:
                raise PermissionError(reason)
            if self.active and owner != self.owner:
                raise PermissionError('Another controller owns the current movement')
            # Set deadline before writing, so transport time cannot extend a lease.
            self.deadline = self.now() + duration
            self.owner = owner
            self.active = True
            try:
                self.write(float(left), float(right))
            except Exception:
                self.stop('transport failure')
                raise
            self.tick()
            return self.status()

    def stop(self, reason='explicit stop'):
        with self.lock:
            self.active = False
            self.deadline = 0.0
            self.owner = None
            self.last_stop = reason
            self.write(0.0, 0.0)

    def disconnect(self, owner):
        with self.lock:
            if self.active and self.owner == owner:
                self.stop('controller disconnected')

    def tick(self):
        with self.lock:
            if not self.active:
                return
            reason = None
            if self.now() >= self.deadline:
                reason = 'movement lease expired'
            elif not self.enabled():
                reason = 'motion disarmed'
            else:
                reason = self.unsafe_reason(None, None)
            if reason:
                self.stop(reason)

    def status(self):
        with self.lock:
            return {'motion_enabled': bool(self.enabled()), 'moving_command_active': self.active,
                    'owner': self.owner, 'last_stop_reason': self.last_stop,
                    'lease_remaining_s': max(0.0, round(self.deadline - self.now(), 3)),
                    'max_speed_m_s': self.MAX_SPEED, 'max_duration_s': self.MAX_DURATION}
