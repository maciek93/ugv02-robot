#!/usr/bin/env python3
"""Collect one matched stationary RGB/depth sample and a human-supplied label."""
import argparse
import hashlib
import json
from pathlib import Path
import secrets
import time
import requests

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--label',choices=('clear','blocked','uncertain'),required=True)
parser.add_argument('--object',required=True)
parser.add_argument('--distance-mm',type=float)
parser.add_argument('--split',choices=('calibration','validation'),required=True)
parser.add_argument('--notes',required=True)
args=parser.parse_args()
state=requests.get('http://127.0.0.1:5001/status',timeout=2).json()
if state.get('moving') or state.get('execution'):
    raise SystemExit('Obstacle samples are stationary; stop before collecting one')
configuration=json.loads((root/'data/active-configuration.json').read_text())
matched=False
for _ in range(25):
    try:
        metadata=json.loads((root/'.runtime/vision.json').read_text())
        rgb=(root/'.runtime/frame.jpg').read_bytes()
        depth=(root/'.runtime/depth.jpg').read_bytes()
        if (hashlib.sha256(rgb).hexdigest()==metadata.get('rgb_sha256')
                and hashlib.sha256(depth).hexdigest()==metadata.get('depth_sha256')
                and time.monotonic()-metadata['mac_updated']<.5):
            matched=True
            break
    except (OSError,ValueError,KeyError): pass
    time.sleep(.02)
if not matched: raise SystemExit('No matched fresh RGB/depth observation; no sample saved')
directory=root/'data/obstacles'/(time.strftime('%Y%m%d-%H%M%S')+'-'+secrets.token_hex(3))
directory.mkdir(parents=True)
(directory/'rgb.jpg').write_bytes(rgb)
(directory/'relative-depth.jpg').write_bytes(depth)
(directory/'sample.json').write_text(json.dumps({'label':args.label,'object':args.object,
    'measured_distance_mm':args.distance_mm,'split':args.split,'human_notes':args.notes,
    'configuration':configuration,'observation':metadata,'depth_units':'relative cue; not metres'},indent=2))
print(directory)
