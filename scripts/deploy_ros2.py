#!/usr/bin/env python3
"""Build the pinned ROS sidecar on the Pi; leave both services disarmed."""
from pathlib import Path
import subprocess
import tarfile
import tempfile

root = Path(__file__).resolve().parents[1]
runtime = root / '.runtime'
ssh = ['ssh', '-S', str(runtime / 'ssh-control'), '-o', 'BatchMode=yes', 'ugvpi@ugvpi.local']
scp = ['scp', '-o', 'ControlPath=' + str(runtime / 'ssh-control'), '-o', 'BatchMode=yes']


def remote(command):
    subprocess.run(ssh + [command], check=True)


remote('sudo systemctl stop ugv-ros2 2>/dev/null || true; sudo systemctl stop ugv-supervised; mkdir -p ~/ugv-ros2 ~/robot-maps')
with tempfile.TemporaryDirectory() as directory:
    archive = Path(directory) / 'ugv-ros2-source.tar.gz'
    with tarfile.open(archive, 'w:gz') as output:
        for path in sorted((root / 'ros2').rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                output.add(path, path.relative_to(root))
        for name in ('metric_map.py', 'metric_mission.py'):
            path = root / 'supervised' / name
            output.add(path, path.relative_to(root))
    subprocess.run(scp + [str(archive), 'ugvpi@ugvpi.local:ugv-ros2-source.tar.gz'], check=True)
    subprocess.run(scp + [str(root / 'deploy/ugv-ros2.service'),
                          'ugvpi@ugvpi.local:robot-maps/ugv-ros2.service'], check=True)
try:
    remote('set -e; tar -xzf ~/ugv-ros2-source.tar.gz -C ~/ugv-ros2; '
           'if ! command -v docker >/dev/null; then sudo apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y docker.io; fi; '
           'sudo docker build -t ugv-ros2:jazzy -f ~/ugv-ros2/ros2/Dockerfile ~/ugv-ros2; '
           'sudo install -m 644 ~/robot-maps/ugv-ros2.service /etc/systemd/system/ugv-ros2.service; '
           'sudo systemctl daemon-reload; sudo systemctl disable ugv-ros2; '
           'sudo systemctl start ugv-ros2')
finally:
    remote('sudo systemctl start ugv-supervised')
print('ROS sidecar built and started; boot autostart disabled. Mapping motion remains disarmed.')
