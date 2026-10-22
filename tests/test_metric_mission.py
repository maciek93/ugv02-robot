from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'supervised'))
from metric_mission import MetricMission


PROFILE = {'version': 2, 'kind': 'lidar-mapping', 'configuration_id': 'test',
           'lidar_serial': 'ld06-test', 'surface': 'level-floor',
           'evidence': 'Measured setup for a simulated mission only',
           'length_m': .5, 'width_m': .4, 'track_m': .4,
           'lidar_x_m': .2, 'lidar_y_m': 0., 'lidar_yaw_rad': 0.,
           'encoder_m_per_unit': .01, 'encoder_signs': [-1, -1],
           'motor_polarity': [-1, -1], 'upper_speed_m_s': .2,
           'upper_turn_rad_s': .8, 'fallback_stop_m': .7, 'margin_m': .05,
           'forward_pwm': 50, 'turn_pwm': 80,
           'forward_m_s_at_pwm': .164, 'turn_rad_s_at_pwm': .571,
           'min_smooth_forward_pwm': 45, 'min_smooth_turn_pwm': 75}
SPEC = {'configuration_id': 'test', 'rooms': ['A', 'B'],
        'doorways': [{'id': 'A-B', 'from_room': 'A', 'to_room': 'B'}],
        'start_room': 'A', 'duration_s': 600, 'travel_m': 10}


class MetricMissionTests(unittest.TestCase):
    def setUp(self):
        self.time = 1.
        self.m = MetricMission(lambda: self.time)
        p = self.m.propose(SPEC, 'session', PROFILE)
        self.m.approve(p['id'], 'session', PROFILE)
        self.identity = {'mission_id': p['id'], 'session': 'session', 'configuration_id': 'test'}
        self.m.observe({'data': {'odl': 0, 'odr': 0}, 'age_s': 0}, PROFILE)

    def test_duplicate_goal_and_renewal_replay(self):
        goal = {**self.identity, 'goal_id': 'g1', 'action': 'follow_route', 'map_version': 'map1'}
        self.m.begin(goal, 'session', PROFILE)
        first = {**self.identity, 'goal_id': 'g1', 'seq': 0, 'left_pwm': 50, 'right_pwm': 50}
        self.m.renew(first, 'session', PROFILE)
        with self.assertRaises(PermissionError): self.m.renew(first, 'session', PROFILE)
        self.m.finish('g1', 'blocked')
        with self.assertRaises(PermissionError): self.m.begin(goal, 'session', PROFILE)
        self.assertIsNotNone(self.m.active)

    def test_expiry_restart_and_quota(self):
        self.time = 602
        with self.assertRaises(PermissionError): self.m.begin({**self.identity, 'goal_id': 'g'}, 'session', PROFILE)
        self.m.cancel('restart')
        self.assertIsNone(self.m.active)
        with self.assertRaises(PermissionError): self.m.renew({**self.identity, 'goal_id': 'g', 'seq': 0}, 'session', PROFILE)

    def test_missed_renewal_and_budget(self):
        self.m.begin({**self.identity, 'goal_id': 'g', 'action': 'drive_distance'}, 'session', PROFILE)
        self.m.renew({**self.identity, 'goal_id': 'g', 'seq': 0, 'left_pwm': 50, 'right_pwm': 50}, 'session', PROFILE)
        self.time += .11
        with self.assertRaises(PermissionError):
            self.m.renew({**self.identity, 'goal_id': 'g', 'seq': 1, 'left_pwm': 50, 'right_pwm': 50}, 'session', PROFILE)

    def test_slow_movement_does_not_spend_quota_at_maximum_speed(self):
        self.m.active['travel_m'] = 1.5
        self.m.begin({**self.identity, 'goal_id': 'slow', 'action': 'drive_distance'}, 'session', PROFILE)
        count = 0
        sample_time = self.time
        for sequence in range(200):
            if sequence % 4 == 0:
                count -= 1  # 0.01 m every 0.2 seconds; 0.05 m/s.
                sample_time = self.time
            self.m.observe({'data': {'odl': count, 'odr': count},
                            'age_s': self.time - sample_time}, PROFILE)
            self.m.renew({**self.identity, 'goal_id': 'slow', 'seq': sequence,
                          'left_pwm': 80, 'right_pwm': 80}, 'session', PROFILE)
            self.time += .05
        self.assertAlmostEqual(self.m.active['traveled_m'], .5, places=2)
        self.assertLess(self.m.active['reserved_m'], 1.3)

    def test_encoder_reset_rejected(self):
        self.m.begin({**self.identity, 'goal_id': 'reset', 'action': 'drive_distance'}, 'session', PROFILE)
        self.time += .2
        with self.assertRaisesRegex(PermissionError, 'Encoder reset'):
            self.m.observe({'data': {'odl': 100, 'odr': 100}, 'age_s': 0}, PROFILE)


if __name__ == '__main__':
    unittest.main()
