#!/usr/bin/env python3
"""Select a saved pose graph for a fresh, disarmed localisation attempt."""
import argparse
import json
import math
import re
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('mission_id')
p.add_argument('x', type=float)
p.add_argument('y', type=float)
p.add_argument('yaw_rad', type=float)
a = p.parse_args()
if not re.fullmatch(r'[0-9a-f]{24}', a.mission_id) or not all(math.isfinite(v) for v in (a.x, a.y, a.yaw_rad)):
    raise SystemExit('Invalid map selection')
payload = json.dumps({'mission_id': a.mission_id, 'initial_pose': [a.x, a.y, a.yaw_rad]})
ssh = ['ssh', '-S', str(root / '.runtime/ssh-control'), '-o', 'BatchMode=yes', 'ugvpi@ugvpi.local']
command = ('set -e; sudo systemctl stop ugv-ros2; '
           'test -f ~/robot-maps/' + a.mission_id + '/survey.posegraph; '
           'test -f ~/robot-maps/' + a.mission_id + '/survey.data; '
           'cat > ~/.config/ugv-supervised/reuse-map.json; '
           'sudo systemctl start ugv-ros2')
subprocess.run(ssh + [command], input=payload, text=True, check=True)
print('Saved map selected. Movement stays disarmed until fresh localisation and mission approval.')
