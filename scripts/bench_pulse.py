#!/usr/bin/env python3
"""One supervised 0.08 m/s, 250 ms wheel pulse. Run only after user readiness."""
import json
from pathlib import Path
import time
import urllib.request

BASE = 'http://127.0.0.1:5000'
ARM = Path('/run/ugv-web/motion-enabled')

def health():
    return json.load(urllib.request.urlopen(BASE + '/health', timeout=2))

def post(path, data):
    request = urllib.request.Request(BASE + path, data=json.dumps(data).encode(),
                                     headers={'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(request, timeout=2))

before = health()
assert before['camera_ready'] and before['controller_connected'], before
assert not before['moving_command_active'] and not before['motion_enabled'], before
assert before['left_speed_m_s'] == before['right_speed_m_s'] == 0, before
print('Before pulse:', before, flush=True)
samples = []
start = time.monotonic()
try:
    ARM.write_text(str(time.monotonic() + 15))
    reply = post('/api/drive', {'left': .08, 'right': .08, 'duration': .25,
                               'owner': 'supervised-bench-pulse'})
    print('Pulse accepted:', reply, flush=True)
    while time.monotonic() - start < 2:
        state = health()
        elapsed = round(time.monotonic() - start, 3)
        samples.append((elapsed, state))
        print(json.dumps({'elapsed_s': elapsed, **state}), flush=True)
        time.sleep(.025)
    peak_left = max(abs(s['left_speed_m_s']) for _, s in samples)
    peak_right = max(abs(s['right_speed_m_s']) for _, s in samples)
    expired = any(s['last_stop_reason'] == 'movement lease expired' and not s['moving_command_active']
                  for _, s in samples)
    after = samples[-1][1]
    assert expired, 'Automatic lease expiry not observed'
    assert after['left_speed_m_s'] == after['right_speed_m_s'] == 0, after
    print(f'PEAK measured wheel speed: left={peak_left} right={peak_right} m/s', flush=True)
    print('PASS: automatic Stop issued; telemetry returned to zero. Physical observation still required.', flush=True)
finally:
    ARM.unlink(missing_ok=True)
    print('Disarmed:', post('/api/stop', {}), flush=True)
