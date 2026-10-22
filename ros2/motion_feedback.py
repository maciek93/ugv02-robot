"""Voltage-independent wheel feedback for the direct-PWM ROS bridge.

The recorded PWM curve is only a starting guess. Encoder displacement updates an
online PWM-to-speed estimate; a slow motor receives more PWM without stop/start
pulses. Mission limits and the guardian still bound every command.
"""
import math


def wrap_delta(previous, current):
    return (current - previous + math.pi) % (2 * math.pi) - math.pi


class AdaptivePWM:
    CAP = 80

    def __init__(self):
        self.gain = [None, None]
        self.measured = [0., 0.]
        self.last_pwm = [0, 0]
        self.last_target = [0., 0.]
        self.new_sample = False
        self.last_motion_at = [None, None]
        self.saturated_since = [None, None]

    def reset(self):
        self.__init__()

    def observe(self, left_m, right_m, dt, now):
        if not .03 <= dt <= .75:
            return
        for side, distance in enumerate((left_m, right_m)):
            speed = distance / dt
            self.measured[side] = speed
            if abs(distance) >= .004:
                self.last_motion_at[side] = now
        self.new_sample = True

    def command(self, linear, angular, profile, now):
        half_turn = angular * profile['track_m'] / 2
        targets = [linear - half_turn, linear + half_turn]
        turning = abs(half_turn) > abs(linear) * .5
        nominal = ((profile['turn_rad_s_at_pwm'] * profile['track_m'] / 2) / profile['turn_pwm']
                   if turning else profile['forward_m_s_at_pwm'] / profile['forward_pwm'])
        minimum = profile['min_smooth_turn_pwm'] if turning else profile['min_smooth_forward_pwm']
        sampled = self.new_sample
        if sampled:
            for side, target in enumerate(targets):
                previous = self.last_pwm[side]
                if (abs(target) < .01 or not previous or
                    target * previous <= 0 or
                    abs(target - self.last_target[side]) > max(.025, abs(self.last_target[side]) * .35)):
                    continue
                speed = self.measured[side]
                if speed * previous > 0 and abs(speed) >= .012:
                    observed = abs(speed / previous)
                    old = self.gain[side] or nominal
                    self.gain[side] = max(nominal * .35, min(nominal * 2., .65 * old + .35 * observed))
                else:
                    self.gain[side] = max(nominal * .35, (self.gain[side] or nominal) * .82)
            self.new_sample = False
        output = []
        for side, target in enumerate(targets):
            if abs(target) < .005:
                output.append(0)
                self.saturated_since[side] = None
                continue
            gain = self.gain[side] or nominal
            wanted = min(self.CAP, max(minimum, round(abs(target) / gain)))
            previous = abs(self.last_pwm[side])
            # One encoder sample changes power by at most six points. This is a
            # smooth continuous correction, never an intervening zero command.
            if previous and target * self.last_pwm[side] > 0:
                if not sampled and abs(target - self.last_target[side]) < .005:
                    wanted = previous
                else:
                    wanted = min(previous + 6, max(previous - 6, wanted))
            pwm = int(math.copysign(wanted, target))
            output.append(pwm)
            if abs(pwm) >= self.CAP - 2:
                if self.saturated_since[side] is None:
                    self.saturated_since[side] = now
            else:
                self.saturated_since[side] = None
        self.last_pwm = output
        self.last_target = targets
        return tuple(output)

    def stalled(self, now):
        """Only report a stall when a requested wheel barely moves at full power."""
        for side in (0, 1):
            since = self.saturated_since[side]
            if (since is not None and now - since >= 1.5 and
                (self.last_motion_at[side] is None or now - self.last_motion_at[side] >= 1.5)):
                return True
        return False
