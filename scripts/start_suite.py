#!/usr/bin/env python3
"""Reusable startup: reuse SSH authentication, idle controller, no automatic movement."""
import json
from pathlib import Path
import subprocess
import time
import requests

root = Path(__file__).resolve().parents[1]
runtime = root / '.runtime'
ssh = ['ssh','-S',str(runtime/'ssh-control'),'-o','BatchMode=yes']
try:
    existing = requests.get('http://127.0.0.1:5001/status',timeout=.3)
    if existing.ok and existing.json().get('gateway_protocol') == 3:
        print('Suite already running. Check status before starting observation.')
        raise SystemExit(0)
    if existing.ok:
        raise SystemExit('Old gateway is running; stop and upgrade it before starting another.')
except requests.RequestException:
    pass
connection = subprocess.run(ssh+['-O','check','ugvpi@ugvpi.local'],capture_output=True)
if connection.returncode:
    subprocess.run(['open','-a','Terminal',str(root/'connect-pi.command')],check=True)
    print('AUTHENTICATION_REQUIRED: enter the Pi password only in Terminal, then run this again.')
    raise SystemExit(2)
subprocess.run(ssh+['ugvpi@ugvpi.local','sudo systemctl start ugv-supervised'],check=True)
log = (runtime/'mac.log').open('a')
process = subprocess.Popen([str(root/'.venv/bin/python'),str(root/'supervised/mac.py')],
                           cwd=root,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                           start_new_session=True)
for _ in range(40):
    if process.poll() is not None:
        raise SystemExit('Gateway exited; inspect .runtime/mac.log. No motion was enabled.')
    try:
        response = requests.get('http://127.0.0.1:5001/status',timeout=.3)
        if response.ok and response.json().get('gateway_protocol') == 3:
            print('Suite ready at http://127.0.0.1:5001/; camera off, motion disarmed.')
            break
    except requests.RequestException:
        pass
    time.sleep(.1)
else:
    process.terminate()
    raise SystemExit('Gateway readiness timeout; startup cancelled.')
