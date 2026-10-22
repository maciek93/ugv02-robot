import tempfile
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'supervised'))
from guardian import Guardian, create_app
from tests.test_supervised import FakeCamera, FakeChassis
from tests.test_metric_mission import PROFILE, SPEC


class MetricGuardianTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.clock = 10.
        now = lambda: self.clock
        self.chassis = FakeChassis(now)
        self.chassis.motor_polarity = (-1, -1)
        self.camera = FakeCamera(Path(self.temp.name), now)
        self.guardian = Guardian(self.chassis, self.camera, Path(self.temp.name), now, wall_now=now)
        self.keys = {'agent': 'agent', 'operator': 'operator', 'vision': 'vision', 'navigation': 'navigation'}
        self.client = create_app(self.guardian, self.keys).test_client()
        self.assertEqual(self.post('/operator/metric-profile', PROFILE, 'operator').status_code, 200)
        self.assertEqual(self.post('/operator/session', {'operation': 'start', 'mode': 'mapping'}, 'operator').status_code, 200)
        self.scan(1)

    def tearDown(self):
        self.guardian.close()
        self.temp.cleanup()

    def post(self, path, body, role='agent'):
        return self.client.post(path, json=body, headers={'X-Robot-Key': self.keys[role]})

    def scan(self, seq):
        self.guardian.heartbeat_at = self.clock
        result = self.post('/metric/scan', {'sequence': seq, 'stamp': self.clock,
                           'frame_id': 'base_laser', 'valid_beams': 100}, 'navigation')
        self.assertEqual(result.status_code, 200)

    def mission(self):
        proposal = self.post('/metric/mission/propose', SPEC).json
        self.assertEqual(self.post('/operator/metric/mission/approve',
                                   {'mission_id': proposal['id']}, 'operator').status_code, 200)
        return {'mission_id': proposal['id'], 'session': self.guardian.policy.session,
                'configuration_id': PROFILE['configuration_id']}

    def test_only_navigation_key_can_renew_and_stop_cancels(self):
        identity = self.mission()
        goal = {**identity, 'goal_id': 'g', 'action': 'drive_distance', 'map_version': 'v1'}
        self.assertEqual(self.post('/metric/navigation/begin', goal, 'navigation').status_code, 200)
        renewal = {**identity, 'goal_id': 'g', 'seq': 0, 'left_pwm': 50, 'right_pwm': 50}
        self.assertEqual(self.post('/metric/navigation/renew', renewal).status_code, 403)
        self.assertEqual(self.post('/metric/navigation/renew', renewal, 'navigation').status_code, 200)
        self.assertEqual(self.chassis.writes[-1], ('pwm', 50, 50))
        self.assertEqual(self.post('/metric/navigation/renew', renewal, 'navigation').status_code, 423)
        self.assertIsNone(self.guardian.metric_mission.active)
        self.assertIn(self.chassis.writes[-1], ((0, 0), ('pwm', 0, 0)))

    def test_stale_scan_blocks_mission_and_restart_has_no_authority(self):
        identity = self.mission()
        self.clock += .6
        self.guardian.heartbeat_at = self.clock
        goal = {**identity, 'goal_id': 'g', 'action': 'follow_route', 'map_version': 'v1'}
        self.assertEqual(self.post('/metric/navigation/begin', goal, 'navigation').status_code, 423)
        self.assertIsNone(self.guardian.metric_mission.active)
        self.post('/operator/session', {'operation': 'pause', 'mode': 'mapping'}, 'operator')
        self.post('/operator/session', {'operation': 'start', 'mode': 'mapping'}, 'operator')
        self.scan(1)
        self.assertEqual(self.post('/metric/navigation/begin', goal, 'navigation').status_code, 423)

    def test_scan_sequence_cannot_hide_a_frozen_capture_time(self):
        frozen = {'sequence': 2, 'stamp': self.clock, 'frame_id': 'base_laser', 'valid_beams': 100}
        self.assertEqual(self.post('/metric/scan', frozen, 'navigation').status_code, 423)
        self.clock += .6
        frozen['sequence'] = 3
        self.assertEqual(self.post('/metric/scan', frozen, 'navigation').status_code, 423)


if __name__ == '__main__':
    unittest.main()
