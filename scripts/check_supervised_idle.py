#!/usr/bin/env python3
"""Stationary live checks; never approves or sends a nonzero motor command."""
import json
from pathlib import Path
import subprocess
import time
import requests

root = Path(__file__).resolve().parents[1]
keys = json.loads((root / '.runtime/pi-keys.json').read_text())
base = 'http://127.0.0.1:5002'
headers = {'X-Robot-Key': keys['agent']}
checks = []

def ready():
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        try:
            result = requests.get(base + '/health', headers=headers, timeout=.3)
            if result.ok:
                return result.json()
        except requests.RequestException:
            pass
        time.sleep(.1)
    raise RuntimeError('Controller did not become ready within four seconds')

def test(name, method, path, expected, payload=None, role='agent'):
    r = requests.request(method, base + path, headers={'X-Robot-Key': keys[role]}, json=payload, timeout=2)
    checks.append({'test': name, 'status': r.status_code, 'expected': expected, 'result': r.json()})
    assert r.status_code == expected, (name, r.status_code, r.json())

ready()
test('idle health', 'GET', '/health', 200)
state = checks[-1]['result']
assert state['state'] == 'idle' and not state['moving'] and not state['approved']
test('idle cannot drive', 'POST', '/pulse', 423, {'proposal_id':'invented'})
test('legacy driving rejected', 'POST', '/api/drive', 403, {})
test('agent cannot approve', 'POST', '/operator/approve', 403, {})
test('agent cannot alter calibration', 'POST', '/operator/profile', 403, {})
test('agent cannot forge perception', 'POST', '/perception', 403, {})
test('vision credential cannot move', 'POST', '/pulse', 403, {'proposal_id':'invented'}, role='vision')
test('Stop is allowed while idle', 'POST', '/stop', 200, {})
ssh = ['ssh','-S',str(root / '.runtime/ssh-control'),'-o','BatchMode=yes','ugvpi@ugvpi.local']
cycles = []
for index in range(3):
    start = time.monotonic()
    r = subprocess.run(ssh + ['sudo systemctl stop ugv-supervised; systemctl show ugv-supervised -p Result -p ExecMainStatus; fuser /dev/video0 2>/dev/null || true'], check=True, text=True, capture_output=True)
    elapsed = time.monotonic() - start
    assert 'Result=success' in r.stdout and 'ExecMainStatus=0' in r.stdout, r.stdout
    assert elapsed < 3, elapsed
    cycles.append({'stop_s_including_ssh': elapsed, 'result': r.stdout.strip()})
    subprocess.run(ssh + ['sudo systemctl start ugv-supervised'], check=True)
    result = ready()
    assert result['state'] == 'idle' and not result['moving'] and not result['approved']
report = {'checks': checks, 'shutdown_cycles': cycles}
(root / 'records/supervised-idle-checks.json').write_text(json.dumps(report, indent=2))
print(json.dumps({'checks_passed':len(checks),'shutdown_cycles':cycles},indent=2))
