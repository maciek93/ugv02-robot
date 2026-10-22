#!/usr/bin/env python3
"""Explicit operator action on the Pi, only for supervised supported bench use."""
import argparse
from pathlib import Path
import time
import urllib.request
import json

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds',type=int,default=60)
parser.add_argument('--disarm',action='store_true')
args=parser.parse_args()
path=Path('/run/ugv-web/motion-enabled')
if args.disarm:
    path.unlink(missing_ok=True)
    request=urllib.request.Request('http://127.0.0.1:5000/api/stop',data=b'{}',headers={'Content-Type':'application/json'})
    print(json.load(urllib.request.urlopen(request,timeout=3)))
else:
    if not 1 <= args.seconds <= 120:
        parser.error('--seconds must be 1–120')
    state=json.load(urllib.request.urlopen('http://127.0.0.1:5000/health',timeout=3))
    if not state['camera_ready'] or not state['controller_connected'] or state['moving_command_active']:
        raise RuntimeError('Need fresh camera/telemetry and stopped command state')
    path.write_text(str(time.monotonic()+args.seconds))
    print(f'Supervised bench control armed for {args.seconds} seconds; hold to drive, release or STOP to stop.')
