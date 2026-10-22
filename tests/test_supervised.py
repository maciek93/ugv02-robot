import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'supervised'))
from guardian import Guardian, create_app
from policy import Policy
from transport import Chassis
import threading


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.
        self.policy = Policy(lambda: self.clock)
        self.policy.start('bench')

    def test_no_approval_no_motion_and_approval_consumed_once(self):
        p = self.policy.propose('pulse_forward', .25, 'frame1')
        with self.assertRaises(PermissionError):
            self.policy.execute(p['id'])
        with self.assertRaises(PermissionError):
            self.policy.approve(p['id'], False)
        self.policy.approve(p['id'], True)
        self.policy.execute(p['id'])
        with self.assertRaises(PermissionError):
            self.policy.execute(p['id'])
        with self.assertRaises(PermissionError):
            self.policy.approve(p['id'], True)
        self.assertEqual(self.policy.pulse('newframe'), (.05, .05, .25))
        self.assertIsNone(self.policy.pulse('nextframe'))
        with self.assertRaises(PermissionError):
            self.policy.execute(p['id'])

    def test_expiry_stop_session_restart_and_new_proposal_invalidate(self):
        for cause in ('expiry', 'stop', 'restart', 'new_proposal'):
            self.setUp()
            p = self.policy.propose('pulse_left', .25, 'frame1')
            self.policy.approve(p['id'], True)
            if cause == 'expiry': self.clock += 31
            if cause == 'stop': self.policy.invalidate('Stop')
            if cause == 'restart': self.policy.start('bench')
            if cause == 'new_proposal': self.policy.propose('pulse_right', .2, 'frame2')
            with self.subTest(cause=cause), self.assertRaises(PermissionError):
                self.policy.execute(p['id'])

    def test_bounds_reversing_and_unvalidated_tour(self):
        for action, amount in [('reverse', .25), ('pulse_forward', 1.01), ('pulse_left', float('nan')),
                               ('pulse_forward', True), ('forward', 50)]:
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.policy.propose(action, amount, 'frame')
        with self.assertRaises(PermissionError):
            self.policy.start('tour')

    def test_floor_sequence_keeps_short_leases_and_stop_cancels_remainder(self):
        self.policy.start('calibration')
        for action, amount in [('pulse_forward', 2.01), ('pulse_left', .26)]:
            with self.assertRaises(ValueError):
                self.policy.propose(action, amount, 'frame')
        proposal = self.policy.propose('pulse_forward', 2, 'frame')
        self.policy.approve(proposal['id'], True)
        self.policy.execute(proposal['id'])
        for i in range(3):
            self.assertEqual(self.policy.pulse(str(i)), (.05, .05, .25))
        self.policy.invalidate('human Stop')
        with self.assertRaises(PermissionError):
            self.policy.pulse('next')
        with self.assertRaises(PermissionError):
            self.policy.execute(proposal['id'])

    def test_manual_commands_are_bounded_and_single_use(self):
        self.policy.start('manual')
        with self.assertRaises(ValueError): self.policy.propose('continuous_left', 10.01, None)
        with self.assertRaises(ValueError): self.policy.propose('reverse', 1, None)
        p = self.policy.propose('continuous_right', 2, None)
        self.policy.approve(p['id'], True)
        self.policy.execute(p['id'])
        with self.assertRaises(PermissionError): self.policy.execute(p['id'])
        self.policy.invalidate('STOP')
        with self.assertRaises(PermissionError): self.policy.execute(p['id'])

    def test_pwm_forward_is_manual_only_and_bounded(self):
        with self.assertRaises(ValueError): self.policy.propose('pwm_forward', 5, None)
        self.policy.start('manual')
        with self.assertRaises(ValueError): self.policy.propose('pwm_forward', 30.01, None)
        with self.assertRaises(ValueError): self.policy.propose('pwm_left', 30.01, None)
        proposal = self.policy.propose('pwm_forward', 5, None)
        self.policy.approve(proposal['id'], True)
        self.policy.execute(proposal['id'])
        with self.assertRaises(PermissionError): self.policy.pulse(None)
        self.policy.invalidate('STOP')
        with self.assertRaises(PermissionError): self.policy.execute(proposal['id'])

    def test_leg_quota_and_new_observation_required(self):
        self.policy.profile = {'upper_speed_m_s': .2, 'upper_turn_deg_s': 60}
        self.policy.start('tour')
        for action, amount in [('forward', 51), ('left', 16)]:
            with self.assertRaises(ValueError):
                self.policy.propose(action, amount, 'frame')
        proposal = self.policy.propose('forward', 50, 'frame')
        self.policy.approve(proposal['id'], True)
        self.policy.execute(proposal['id'])
        self.policy.pulse('frame2', 0)
        with self.assertRaises(PermissionError):
            self.policy.pulse('frame2', 0)
        total = .25
        for i in range(30):
            command = self.policy.pulse('frame' + str(i+3), total * 20)
            if command is None: break
            total += command[2]
        self.assertLessEqual(total, 2.5)
        self.assertIsNone(self.policy.execution)


class FakeChassis:
    def __init__(self, clock):
        self.clock = clock
        self.writes = []
        self.fail = False
        self.old = False

    def status(self):
        return {'data': {'L': 0, 'R': 0, 'v': 1200, 'odl': 0, 'odr': 0},
                'battery_voltage_v': 12., 'age_s': 5 if self.old else 0, 'error': None}

    def wheels(self, left, right):
        self.writes.append((left, right))
        if self.fail: raise OSError('simulated UART fault')

    def pwm(self, left, right):
        self.writes.append(('pwm', left, right))
        if self.fail: raise OSError('simulated UART fault')

    def close(self): pass


class TransportTests(unittest.TestCase):
    def test_pwm_is_bounded_and_stop_clears_pwm_before_speed_mode(self):
        class Port:
            def __init__(self): self.messages = []
            def write(self, payload):
                self.messages.append(json.loads(payload))
                return len(payload)
        chassis = object.__new__(Chassis)
        chassis.port = Port()
        chassis.write_lock = threading.Lock()
        chassis.motor_polarity = (-1, -1)
        chassis.motor_mode = 'speed'
        with self.assertRaises(ValueError): chassis.pwm(81, 0)
        chassis.pwm(50, 50)
        self.assertEqual(chassis.port.messages[-1], {'T':11, 'L':-50, 'R':-50})
        # Right turn (left +80, right -80): firmware L (right) gets 80, firmware R (left) gets -80
        chassis.pwm(80, -80)
        self.assertEqual(chassis.port.messages[-1], {'T':11, 'L':80, 'R':-80})
        # Left turn (left -80, right +80): firmware L (right) gets -80, firmware R (left) gets 80
        chassis.pwm(-80, 80)
        self.assertEqual(chassis.port.messages[-1], {'T':11, 'L':-80, 'R':80})
        chassis.wheels(0, 0)
        self.assertEqual(chassis.port.messages[-2:], [{'T':11, 'L':0, 'R':0},
                                                      {'T':1, 'L':0, 'R':0}])
        self.assertEqual(chassis.motor_mode, 'speed')
        # Test telemetry remapping
        chassis.data_lock = threading.Lock()
        chassis.data = {'odl': 10, 'odr': 20, 'L': 5, 'R': 6, 'v': 1100}
        chassis.updated = time.monotonic()
        chassis.error = None
        st = chassis.status()
        self.assertEqual(st['data']['odl'], 20)
        self.assertEqual(st['data']['odr'], 10)
        self.assertEqual(st['data']['L'], 6)
        self.assertEqual(st['data']['R'], 5)


class FakeCamera:
    def __init__(self, directory, clock):
        self.directory, self.clock = directory, clock
        self.bad = False
        self.sequence = 0
        self.metadata = None

    def start(self): self.fresh()
    def close(self): self.bad = True

    def fresh(self):
        self.bad = False
        self.sequence += 1
        self.metadata = {'frame_id': f'fake-{self.sequence}', 'captured': self.clock(), 'quality_reason': None}
        (self.directory / (self.metadata['frame_id'] + '.jpg')).write_bytes(b'JPEG')
        (self.directory / (self.metadata['frame_id'] + '.json')).write_text(json.dumps(self.metadata))

    def read(self): return {'error': 'camera unplugged'} if self.bad else self.metadata


class GuardianTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.clock = 10.
        now = lambda: self.clock
        self.camera = FakeCamera(Path(self.temp.name), now)
        self.chassis = FakeChassis(now)
        self.guardian = Guardian(self.chassis, self.camera, Path(self.temp.name), now)
        self.keys = {'agent': 'test-agent', 'operator': 'test-human', 'vision': 'test-vision'}
        self.client = create_app(self.guardian, self.keys).test_client()
        self.post('/operator/session', {'operation': 'start', 'mode': 'bench'}, 'operator')
        self.fresh()

    def tearDown(self):
        self.chassis.fail = False
        self.guardian.close()
        self.temp.cleanup()

    def post(self, path, data, role='agent'):
        return self.client.post(path, json=data, headers={'X-Robot-Key': self.keys[role]})

    def fresh(self):
        self.clock += .01
        self.camera.fresh()
        self.guardian.heartbeat_at = self.clock
        payload = {**self.camera.metadata, 'session': self.guardian.policy.session,
                   'depth_ready': True, 'near_score': 10, 'hazard': None,
                   'error': None, 'signature': [100]*48}
        payload.pop('quality_reason')
        self.assertEqual(self.post('/perception', payload, 'vision').status_code, 200)

    def approved_job(self):
        proposal = self.post('/propose', {'action': 'pulse_forward', 'value': .25}).json
        self.assertEqual(self.post('/operator/approve', {'proposal_id': proposal['id'], 'clearance': True}, 'operator').status_code, 200)
        self.assertEqual(self.post('/execute', {'proposal_id': proposal['id']}).status_code, 200)
        self.clock += .11
        self.fresh()
        self.assertEqual(self.post('/pulse', {'proposal_id': proposal['id']}).status_code, 200)
        return proposal['id']

    def test_continuous_renewal_has_fixed_end_and_stop_invalidates(self):
        self.post('/operator/session', {'operation':'pause', 'mode':'calibration'}, 'operator')
        self.post('/operator/session', {'operation':'start', 'mode':'calibration'}, 'operator')
        self.fresh()
        p = self.post('/propose', {'action':'continuous_forward', 'value':2}).json
        self.assertEqual(self.post('/operator/approve', {'proposal_id':p['id'], 'clearance':True}, 'operator').status_code, 200)
        self.assertEqual(self.post('/execute', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        end = self.guardian.continuous_end
        self.assertLessEqual(self.guardian.deadline-self.clock, .25)
        self.clock += .1
        self.fresh()
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.guardian.continuous_end, end)
        self.post('/stop', {})
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 423)
        self.assertEqual(self.chassis.writes[-1], (0,0))

    def test_extended_forward_trial_retains_a_fixed_end_and_short_lease(self):
        self.post('/operator/session', {'operation':'pause', 'mode':'calibration'}, 'operator')
        self.post('/operator/session', {'operation':'start', 'mode':'calibration'}, 'operator')
        self.fresh()
        self.assertEqual(self.post('/propose', {'action':'continuous_forward', 'value':30.01}).status_code, 400)
        p = self.post('/propose', {'action':'continuous_forward', 'value':30}).json
        self.assertEqual(p['motor_budget_s'], 30)
        self.assertEqual(self.post('/operator/approve', {'proposal_id':p['id'], 'clearance':True}, 'operator').status_code, 200)
        self.assertEqual(self.post('/execute', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertLessEqual(self.guardian.deadline-self.clock, .25)
        self.assertAlmostEqual(self.guardian.continuous_end-self.clock, 30, places=2)
        self.post('/stop', {})
        self.assertEqual(self.chassis.writes[-1], (0,0))

    def test_manual_drive_ignores_perception_and_viewer_but_stop_cancels(self):
        self.post('/operator/session', {'operation':'pause', 'mode':'bench'}, 'operator')
        self.post('/operator/session', {'operation':'start', 'mode':'manual'}, 'operator')
        self.camera.bad = True
        self.chassis.old = True
        self.guardian.heartbeat_at = 0
        self.guardian.vision = {'hazard':'Visible person', 'received':0, 'captured':0}
        self.assertIsNone(self.guardian.unsafe())
        p = self.post('/propose', {'action':'continuous_forward', 'value':27}).json
        self.assertEqual(self.post('/operator/approve', {'proposal_id':p['id'], 'clearance':True}, 'operator').status_code, 200)
        self.assertEqual(self.post('/execute', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.chassis.writes[-1], (.10,.10))
        self.assertLessEqual(self.guardian.deadline-self.clock, .25)
        write_count = len(self.chassis.writes)
        for _ in range(9):
            self.clock += .1
            self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(len(self.chassis.writes), write_count)
        self.clock += .11
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(len(self.chassis.writes), write_count + 1)
        self.post('/stop', {})
        self.assertEqual(self.chassis.writes[-1], (0,0))
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 423)

    def test_manual_pwm_refreshes_heartbeat_and_stop_sends_zero(self):
        self.post('/operator/session', {'operation':'pause', 'mode':'bench'}, 'operator')
        self.post('/operator/session', {'operation':'start', 'mode':'manual'}, 'operator')
        p = self.post('/propose', {'action':'pwm_forward', 'value':5}).json
        self.post('/operator/approve', {'proposal_id':p['id'], 'clearance':True}, 'operator')
        self.post('/execute', {'proposal_id':p['id']})
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.chassis.writes[-1], ('pwm', 50, 50))
        count = len(self.chassis.writes)
        self.clock += .1
        self.post('/renew', {'proposal_id':p['id']})
        self.assertEqual(len(self.chassis.writes), count)
        for _ in range(10):
            self.clock += .091
            self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.chassis.writes[-1], ('pwm', 50, 50))
        self.post('/stop', {})
        self.assertEqual(self.chassis.writes[-1], (0, 0))
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 423)

    def test_manual_pwm_turn_uses_opposing_wheels(self):
        self.post('/operator/session', {'operation':'pause', 'mode':'bench'}, 'operator')
        self.post('/operator/session', {'operation':'start', 'mode':'manual'}, 'operator')
        p = self.post('/propose', {'action':'pwm_left', 'value':2.5}).json
        self.post('/operator/approve', {'proposal_id':p['id'], 'clearance':True}, 'operator')
        self.post('/execute', {'proposal_id':p['id']})
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.chassis.writes[-1], ('pwm', -50, 50))
        self.post('/stop', {})
        self.assertEqual(self.chassis.writes[-1], (0, 0))

    def test_manual_pwm_spin_drives_both_sides_equally(self):
        self.post('/operator/session', {'operation':'pause', 'mode':'bench'}, 'operator')
        self.post('/operator/session', {'operation':'start', 'mode':'manual'}, 'operator')
        p = self.post('/propose', {'action':'pwm_spin_right', 'value':5}).json
        self.post('/operator/approve', {'proposal_id':p['id'], 'clearance':True}, 'operator')
        self.post('/execute', {'proposal_id':p['id']})
        self.assertEqual(self.post('/renew', {'proposal_id':p['id']}).status_code, 200)
        self.assertEqual(self.chassis.writes[-1], ('pwm', 80, -80))
        self.post('/stop', {})
        self.assertEqual(self.chassis.writes[-1], (0, 0))
        p2 = self.post('/propose', {'action':'pwm_spin_left', 'value':5}).json
        self.post('/operator/approve', {'proposal_id':p2['id'], 'clearance':True}, 'operator')
        self.post('/execute', {'proposal_id':p2['id']})
        self.assertEqual(self.post('/renew', {'proposal_id':p2['id']}).status_code, 200)
        self.assertEqual(self.chassis.writes[-1], ('pwm', -80, 80))
        self.post('/stop', {})
        self.assertEqual(self.chassis.writes[-1], (0, 0))

    def test_continuous_deadline_cannot_be_extended_by_late_renewal(self):
        self.post('/operator/session', {'operation':'pause', 'mode':'calibration'}, 'operator')
        self.post('/operator/session', {'operation':'start', 'mode':'calibration'}, 'operator')
        self.fresh()
        p = self.post('/propose', {'action':'continuous_forward', 'value':2}).json
        self.post('/operator/approve', {'proposal_id':p['id'], 'clearance':True}, 'operator')
        self.post('/execute', {'proposal_id':p['id']})
        self.post('/renew', {'proposal_id':p['id']})
        self.clock += .26
        self.fresh()
        self.post('/renew', {'proposal_id':p['id']})
        self.assertIsNone(self.guardian.policy.execution)
        self.assertEqual(self.chassis.writes[-1], (0,0))
    def test_agent_cannot_approve_change_profile_forge_vision_or_bypass(self):
        for path in ('/operator/approve', '/operator/profile', '/operator/session', '/operator/heartbeat', '/perception'):
            self.assertEqual(self.post(path, {}).status_code, 403)
        for path in ('/api/drive', '/json', '/send_command'):
            self.assertEqual(self.post(path, {}).status_code, 403)
        self.assertEqual(self.post('/pulse', {'proposal_id': 'invented'}).status_code, 423)
        self.assertTrue(all(left == right == 0 for left, right in self.chassis.writes))

    def test_watchdog_stops_without_client_when_lease_camera_or_connection_lost(self):
        for cause in ('lease', 'camera', 'heartbeat', 'telemetry', 'perception'):
            if cause != 'lease': self.tearDown(); self.setUp()
            self.approved_job()
            self.assertEqual(self.chassis.writes[-1], (.05, .05))
            if cause == 'lease': self.clock += .26
            if cause == 'camera': self.camera.bad = True
            if cause == 'heartbeat': self.clock += .61
            if cause == 'telemetry': self.chassis.old = True
            if cause == 'perception': self.guardian.vision['error'] = 'worker failed'
            time.sleep(.06)
            with self.subTest(cause=cause):
                self.assertEqual(self.chassis.writes[-1], (0, 0))
                self.assertFalse(self.guardian.deadline)

    def test_stop_cancels_remaining_leg_and_previous_approval(self):
        proposal = self.approved_job()
        self.assertEqual(self.post('/stop', {}).status_code, 200)
        self.assertEqual(self.post('/pulse', {'proposal_id': proposal}).status_code, 423)
        self.assertEqual(self.post('/execute', {'proposal_id': proposal}).status_code, 423)
        self.assertEqual(self.chassis.writes[-1], (0,0))

    def test_transport_failure_does_not_leave_active_command(self):
        self.chassis.fail = True
        # Both initial command and zero write fail; watchdog must keep attempting zero.
        proposal = self.post('/propose', {'action': 'pulse_forward', 'value': .25}).json
        self.post('/operator/approve', {'proposal_id': proposal['id'], 'clearance': True}, 'operator')
        self.post('/execute', {'proposal_id': proposal['id']})
        self.clock += .11
        self.fresh()
        self.post('/pulse', {'proposal_id': proposal['id']})
        self.assertFalse(self.guardian.deadline)
        self.chassis.fail = False
        time.sleep(.06)
        self.assertEqual(self.chassis.writes[-1], (0,0))

    def test_observation_replay_and_forged_capture_time_rejected(self):
        payload = {**self.guardian.vision}
        payload.pop('received')
        self.assertEqual(self.post('/perception', payload, 'vision').status_code, 423)
        payload['captured'] += .001
        self.clock += .002
        self.assertEqual(self.post('/perception', payload, 'vision').status_code, 423)

    def test_scene_change_invalidates_approval(self):
        proposal = self.post('/propose', {'action':'pulse_forward','value':.25}).json
        self.post('/operator/approve', {'proposal_id':proposal['id'],'clearance':True}, 'operator')
        self.fresh()
        self.guardian.vision['signature'] = [180]*48
        self.assertEqual(self.post('/execute', {'proposal_id':proposal['id']}).status_code, 423)
        self.assertIsNone(self.guardian.policy.approval)

    def test_sensor_provider_receives_active_direction(self):
        self.approved_job()
        seen = []
        self.guardian.sensor_providers.append(lambda l,r: seen.append((l,r)) or 'test forward obstacle')
        time.sleep(.06)
        self.assertIn((.05,.05), seen)
        self.assertEqual(self.chassis.writes[-1], (0,0))

    def test_pause_closes_camera_and_restart_disarms(self):
        proposal = self.approved_job()
        self.post('/operator/session', {'operation':'pause','mode':'bench'}, 'operator')
        self.assertTrue(self.camera.bad)
        self.assertEqual(self.guardian.policy.state, 'paused')
        self.post('/operator/session', {'operation':'resume','mode':'bench'}, 'operator')
        self.assertEqual(self.post('/execute', {'proposal_id':proposal}).status_code, 423)
        self.assertFalse(self.guardian.deadline)


if __name__ == '__main__': unittest.main()
