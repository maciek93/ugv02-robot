import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deploy'))
from motion_guard import MotionGuard

class GuardTests(unittest.TestCase):
    def setUp(self):
        self.time = 10.0
        self.armed = True
        self.reason = None
        self.writes = []
        self.guard = MotionGuard(lambda l, r: self.writes.append((l, r)),
                                 lambda: self.armed, lambda l, r: self.reason,
                                 now=lambda: self.time)

    def test_speed_and_duration_limits_and_nonfinite_rejected(self):
        for l, r, d in [(0.11, 0, .2), (0, -.11, .2), (float('nan'), 0, .2),
                        (0, float('inf'), .2), (True, 0, .2), (.08, .08, .36),
                        (.08, .08, 0), (.08, .08, float('nan'))]:
            with self.subTest(values=(l, r, d)), self.assertRaises(ValueError):
                self.guard.drive(l, r, d, 'operator')
        self.assertEqual(self.writes, [])

    def test_disarmed_and_stale_observation_rejected(self):
        self.armed = False
        with self.assertRaises(PermissionError):
            self.guard.drive(.08, .08, .2, 'operator')
        self.armed = True
        self.reason = 'Camera stale'
        with self.assertRaises(PermissionError):
            self.guard.drive(.08, .08, .2, 'operator')
        self.assertEqual(self.writes, [])

    def test_deadman_expires_without_client_stop(self):
        self.guard.drive(.08, .08, .2, 'operator')
        self.time += .21
        self.guard.tick()
        self.assertEqual(self.writes, [(.08, .08), (0., 0.)])
        self.assertFalse(self.guard.active)

    def test_disconnect_and_other_owner(self):
        self.guard.drive(.08, .08, .2, 'operator')
        with self.assertRaises(PermissionError):
            self.guard.drive(.05, .05, .2, 'other')
        self.guard.disconnect('other')
        self.assertTrue(self.guard.active)
        self.guard.disconnect('operator')
        self.assertEqual(self.writes[-1], (0., 0.))

    def test_explicit_stop_works_when_disarmed(self):
        self.guard.drive(.08, .08, .2, 'operator')
        self.armed = False
        self.guard.stop()
        self.assertEqual(self.writes[-1], (0., 0.))

    def test_loss_of_observation_or_arming_stops_active_motion(self):
        for cause in ('stale', 'disarmed'):
            with self.subTest(cause=cause):
                self.setUp()
                self.guard.drive(.08, .08, .2, 'operator')
                if cause == 'stale': self.reason = 'Telemetry stale'
                else: self.armed = False
                self.guard.tick()
                self.assertEqual(self.writes[-1], (0., 0.))

    def test_transport_failure_attempts_stop(self):
        def write(l, r):
            self.writes.append((l, r))
            if l or r: raise OSError('Transport failure')
        self.guard.write = write
        with self.assertRaises(OSError):
            self.guard.drive(.08, .08, .2, 'operator')
        self.assertEqual(self.writes[-1], (0., 0.))
        self.assertFalse(self.guard.active)

    def test_slow_transport_does_not_extend_lease(self):
        def write(l, r):
            self.writes.append((l, r))
            if l or r: self.time += .4
        self.guard.write = write
        self.guard.drive(.08, .08, .2, 'operator')
        self.assertEqual(self.writes, [(.08, .08), (0., 0.)])
        self.assertFalse(self.guard.active)

if __name__ == '__main__':
    unittest.main()
