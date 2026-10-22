#!/usr/bin/env python3
"""On-Pi API checks while disarmed; no successful movement requests."""
import json
import urllib.request
import urllib.error

BASE = 'http://127.0.0.1:5000'
health = json.load(urllib.request.urlopen(BASE + '/health', timeout=3))
assert health['camera_ready'] and health['controller_connected'], health
assert health['mode'] == 'supervised-bench' and not health['motion_enabled'], health
assert not health['moving_command_active'], health
print('Fresh camera/telemetry; motion disarmed:', health)
for payload, expected in [
    ({'left': .08, 'right': .08, 'duration': .25, 'owner': 'check-disarmed'}, 423),
    ({'left': .11, 'right': .08, 'duration': .25, 'owner': 'check-bounds'}, 400),
    ({'left': .08, 'right': .08, 'duration': 2, 'owner': 'check-duration'}, 400),
    ({'T': 11, 'L': 100, 'R': 100}, 400),
]:
    request = urllib.request.Request(BASE + '/api/drive', data=json.dumps(payload).encode(),
                                     headers={'Content-Type': 'application/json'})
    try:
        urllib.request.urlopen(request, timeout=3)
        raise AssertionError('Expected rejection')
    except urllib.error.HTTPError as exc:
        assert exc.code == expected, (exc.code, expected)
        print('Rejected:', exc.code, exc.read().decode().strip())
print('PASS: disarmed API and bounds; no movement accepted')
