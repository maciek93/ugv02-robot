#!/usr/bin/env python3
"""Run on the Pi: real-frame checks and blocked command checks, no movement."""
import concurrent.futures
import json
import time
import urllib.error
import urllib.parse
import urllib.request
import cv2
import numpy as np
import socketio

base = 'http://127.0.0.1:5000'

def get(path):
    return urllib.request.urlopen(base + path, timeout=5)

def stream_check():
    deadline = time.monotonic() + 10
    count = 0
    data = b''
    with get('/video_feed') as response:
        while time.monotonic() < deadline:
            data += response.read(8192)
            while True:
                start = data.find(b'\xff\xd8')
                end = data.find(b'\xff\xd9', start + 2)
                if start < 0 or end < 0:
                    break
                jpeg = data[start:end + 2]
                data = data[end + 2:]
                frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                assert frame is not None and frame.shape[:2] == (480, 640)
                count += 1
    assert count >= 50, f'Too few decoded frames: {count}'
    return count

with get('/health') as response:
    health = json.load(response)
assert health['camera_ready'] is True and health['motion_enabled'] is False, health
if health['mode'] == 'stationary':
    assert health['controller_connected'] is False, health
elif health['mode'] == 'telemetry-only':
    assert health['controller_connected'] is True, health
    assert 8 < health['battery_voltage_v'] < 13, health
    assert health['telemetry_age_s'] < 1.5, health
    assert health['left_speed_m_s'] == health['right_speed_m_s'] == 0, health
else:
    raise AssertionError(f'Unexpected operating mode: {health}')
print('Health:', health)
with get('/snapshot.jpg') as response:
    assert response.headers['Cache-Control'] == 'no-store'
    jpeg = response.read()
frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
assert frame is not None and frame.shape[:2] == (480, 640)
print('Snapshot decoded:', frame.shape)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
    results = list(executor.map(lambda _: stream_check(), range(2)))
print('Two concurrent 10-second streams decoded frames:', results)
data = urllib.parse.urlencode({'command': 'base -c {"T":130}'}).encode()
try:
    urllib.request.urlopen(base + '/send_command', data=data, timeout=5)
    raise AssertionError('Command-line route must reject commands')
except urllib.error.HTTPError as exc:
    assert exc.code == 423, exc.code
print('Command-line route blocked: HTTP 423')
client = socketio.Client()
client.connect(base, namespaces=['/json'])
reply = client.call('json', {'T': 1, 'L': 0, 'R': 0}, namespace='/json', timeout=5)
assert reply['ok'] is False, reply
client.disconnect()
print('Socket command blocked:', reply)
print('PASS: stationary video/command-gate checks; no physical motion or Stop verification')
