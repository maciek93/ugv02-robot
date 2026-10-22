#!/usr/bin/env python3
"""Codex tools for measured LiDAR maps and approved metric missions."""
import argparse
import json
from pathlib import Path
import requests

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
sub = parser.add_subparsers(dest='command', required=True)
for name in ('view-lidar-scan', 'view-depth-grid', 'capture-view', 'status', 'pause', 'stop', 'finish', 'report'):
    sub.add_parser(name)
for name in ('propose', 'preview-route', 'follow-route', 'navigate-to', 'annotate'):
    sub.add_parser(name).add_argument('file', type=Path)
sub.add_parser('start').add_argument('mission_id')
for name in ('drive-distance', 'turn-angle'):
    sub.add_parser(name).add_argument('value', type=float)
view = sub.add_parser('view-metric-map')
view.add_argument('--center-x', type=float)
view.add_argument('--center-y', type=float)
view.add_argument('--width-m', type=float, default=4.)
view.add_argument('--height-m', type=float, default=4.)
inspect = sub.add_parser('inspect-map-region')
inspect.add_argument('center_x', type=float)
inspect.add_argument('center_y', type=float)
inspect.add_argument('--width-m', type=float, default=2.)
inspect.add_argument('--height-m', type=float, default=2.)
args = parser.parse_args()
command = args.command.replace('-', '_')
path = '/agent/metric/' + ('mission/' + command if command in ('propose', 'start') else command)
body = {}
if hasattr(args, 'file'):
    body = json.loads(args.file.read_text())
elif command == 'start':
    body = {'mission_id': args.mission_id}
elif command in ('drive_distance', 'turn_angle'):
    body = {'value': args.value}
elif command == 'view_metric_map':
    body = {k: getattr(args, k) for k in ('center_x', 'center_y', 'width_m', 'height_m')
            if getattr(args, k) is not None}
elif command == 'inspect_map_region':
    body = {k: getattr(args, k) for k in ('center_x', 'center_y', 'width_m', 'height_m')}
method = requests.get if command in ('view_lidar_scan', 'view_depth_grid', 'status', 'report') else requests.post
key = (ROOT / '.runtime/agent-key').read_text().strip()
kwargs = {'headers': {'X-Agent-Key': key}, 'timeout': 10}
if method is requests.post:
    kwargs['json'] = body
response = method('http://127.0.0.1:5001' + path, **kwargs)
print(json.dumps(response.json(), indent=2))
raise SystemExit(0 if response.ok else 1)
