#!/usr/bin/env python3
"""Deploy reviewed session controller without activating camera or nonzero motors."""
import json
from pathlib import Path
import secrets
import subprocess

root = Path(__file__).resolve().parents[1]
runtime = root / '.runtime'
runtime.mkdir(mode=0o700, exist_ok=True)
key_file = runtime / 'pi-keys.json'
if not key_file.exists():
    key_file.write_text(json.dumps({role: secrets.token_urlsafe(32) for role in ('agent', 'operator', 'vision', 'navigation')}))
# Upgrade credentials without replacing existing operator/agent keys.
keys = json.loads(key_file.read_text())
if 'navigation' not in keys:
    keys['navigation'] = secrets.token_urlsafe(32)
    key_file.write_text(json.dumps(keys))
key_file.chmod(0o600)
ssh = ['ssh', '-S', str(runtime / 'ssh-control'), '-o', 'BatchMode=yes', 'ugvpi@ugvpi.local']
scp = ['scp', '-o', 'ControlPath=' + str(runtime / 'ssh-control'), '-o', 'BatchMode=yes']

def remote(command):
    subprocess.run(ssh + [command], check=True)

remote('if systemctl cat ugv-supervised >/dev/null 2>&1; then sudo systemctl stop ugv-supervised; fi')
remote('mkdir -p ~/ugv-supervised ~/.config/ugv-supervised ~/robot-records; chmod 700 ~/.config/ugv-supervised')
files = [str(root / 'supervised' / name) for name in ('guardian.py', 'policy.py', 'camera.py', 'transport.py', 'mission.py', 'metric_mission.py', 'explore_profile.py')]
subprocess.run(scp + files + ['ugvpi@ugvpi.local:ugv-supervised/'], check=True)
subprocess.run(scp + [str(key_file), 'ugvpi@ugvpi.local:.config/ugv-supervised/keys.json'], check=True)
subprocess.run(scp + [str(root / 'deploy/ugv-supervised.service'), 'ugvpi@ugvpi.local:robot-records/ugv-supervised.service'], check=True)
remote('set -e; chmod 600 ~/.config/ugv-supervised/keys.json; '
       '~/ugv_rpi/ugv-env/bin/python -m compileall -q ~/ugv-supervised; '
       'sudo systemctl stop ugv-web; '
       'if systemctl cat ugv-supervised >/dev/null 2>&1; then sudo systemctl stop ugv-supervised; fi; '
       'sudo systemctl disable ugv-web; '
       'if [ -f /etc/systemd/system/ugv-web.service ] && [ ! -L /etc/systemd/system/ugv-web.service ]; then '
       'sudo cp /etc/systemd/system/ugv-web.service ~/robot-records/ugv-web-retired.service; '
       'sudo rm /etc/systemd/system/ugv-web.service; fi; '
       'sudo systemctl mask ugv-web; '
       'sudo install -m 644 ~/robot-records/ugv-supervised.service /etc/systemd/system/ugv-supervised.service; '
       'sudo systemctl daemon-reload; sudo systemctl disable ugv-supervised; '
       'sudo systemctl start ugv-supervised; '
       'systemctl is-active ugv-supervised; systemctl is-enabled ugv-supervised || true; '
       'fuser /dev/video0 2>/dev/null || true')
print('Deployed: controller idle, camera off, boot startup disabled; old service masked.')
