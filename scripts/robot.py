#!/usr/bin/env python3
"""Agent tools intentionally have no approve/start/calibration operation."""
import argparse
import base64
import json
from pathlib import Path
import requests

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
sub = parser.add_subparsers(dest='operation', required=True)
observe = sub.add_parser('observe')
observe.add_argument('--save', type=Path)
propose = sub.add_parser('propose')
propose.add_argument('action', choices=('continuous_forward', 'continuous_left', 'continuous_right', 'pwm_forward', 'pwm_left', 'pwm_right', 'pwm_spin_right', 'pwm_spin_left', 'pulse_forward', 'pulse_left', 'pulse_right', 'forward', 'left', 'right'))
propose.add_argument('value', type=float)
execute = sub.add_parser('execute')
execute.add_argument('proposal_id')
sub.add_parser('stop')
sub.add_parser('end')
sub.add_parser('status')
args = parser.parse_args()
key = (ROOT / '.runtime/agent-key').read_text().strip()
url = 'http://127.0.0.1:5001/agent/' + args.operation
data = {'action': args.action, 'value': args.value} if args.operation == 'propose' else {'proposal_id': args.proposal_id} if args.operation == 'execute' else {}
response = requests.get(url, headers={'X-Agent-Key': key}, timeout=3) if args.operation in ('observe','status') else requests.post(url, headers={'X-Agent-Key': key}, json=data, timeout=3)
result = response.json()
if 'jpeg' in result:
    jpeg = base64.b64decode(result.pop('jpeg'))
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        args.save.write_bytes(jpeg)
        result['saved_image'] = str(args.save.resolve())
print(json.dumps(result, indent=2))
raise SystemExit(0 if response.ok else 1)
