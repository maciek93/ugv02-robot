"""Replay variable motor response without ROS or moving the chassis."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2'))
from motion_feedback import AdaptivePWM, wrap_delta


PROFILE = {
    'track_m': .41,
    'forward_m_s_at_pwm': .164,
    'forward_pwm': 50,
    'turn_rad_s_at_pwm': .571,
    'turn_pwm': 80,
    'min_smooth_forward_pwm': 40,
    'min_smooth_turn_pwm': 65,
}


class FeedbackTests(unittest.TestCase):
    def test_voltage_drop_increases_continuous_pwm_then_recovers(self):
        controller = AdaptivePWM()
        t = 0.
        initial = controller.command(.15, 0, PROFILE, t)
        self.assertEqual(initial[0], initial[1])
        self.assertEqual(controller.command(.15, 0, PROFILE, t + .05), initial)
        for _ in range(9):
            t += .2
            # The same motor power produces less travel as the battery sags.
            controller.observe(.017, .017, .2, t)
            current = controller.command(.15, 0, PROFILE, t)
            self.assertGreater(current[0], 0)
            self.assertGreater(current[1], 0)
        self.assertGreater(current[0], initial[0] + 10)
        self.assertLessEqual(current[0], 80)
        sag_pwm = current[0]
        for _ in range(12):
            t += .2
            # Once voltage recovers, the same PWM overspeeds the wheels.
            controller.observe(.04, .04, .2, t)
            current = controller.command(.15, 0, PROFILE, t)
        self.assertLess(current[0], sag_pwm)
        self.assertEqual(controller.command(0, 0, PROFILE, t), (0, 0))

    def test_full_power_with_barely_moving_wheel_stalls(self):
        controller = AdaptivePWM()
        t = 0.
        controller.command(.2, 0, PROFILE, t)
        for _ in range(30):
            t += .2
            controller.observe(0, 0, .2, t)
            controller.command(.2, 0, PROFILE, t)
            if controller.stalled(t):
                break
        self.assertTrue(controller.stalled(t))
        self.assertEqual(controller.last_pwm, [80, 80])

    def test_slow_but_real_motion_is_not_voltage_cutoff(self):
        controller = AdaptivePWM()
        t = 0.
        controller.command(.2, 0, PROFILE, t)
        for _ in range(30):
            t += .2
            controller.observe(.006, .006, .2, t)
            controller.command(.2, 0, PROFILE, t)
        self.assertFalse(controller.stalled(t))

    def test_opposing_wheels_and_wraparound(self):
        controller = AdaptivePWM()
        left, right = controller.command(0, .5, PROFILE, 0)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)
        self.assertAlmostEqual(wrap_delta(math.radians(179), math.radians(-179)), math.radians(2))
        self.assertAlmostEqual(wrap_delta(math.radians(-179), math.radians(179)), -math.radians(2))


if __name__ == '__main__':
    unittest.main()
