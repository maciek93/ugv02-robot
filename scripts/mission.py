#!/usr/bin/env python3
"""Codex-facing exploration tools. Approval belongs to chat_control.py."""
import argparse
import json
from pathlib import Path
import requests
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
a=s.add_parser('propose');a.add_argument('spec',type=Path)
a=s.add_parser('start');a.add_argument('mission_id')
a=s.add_parser('select');a.add_argument('candidate_id');a.add_argument('--doorway-id')
a=s.add_parser('annotate');a.add_argument('file',type=Path)
for c in ('candidates','status','finish','report'):s.add_parser(c)
args=p.parse_args();data={}
if args.command=='propose':
    data=json.loads(args.spec.read_text());data.setdefault('duration_s',600);data.setdefault('travel_m',10)
elif args.command=='start':data={'mission_id':args.mission_id}
elif args.command=='select':
    data={'candidate_id':args.candidate_id}
    if args.doorway_id:data['doorway_id']=args.doorway_id
elif args.command=='annotate':data=json.loads(args.file.read_text())
headers={'X-Agent-Key':(ROOT/'.runtime/agent-key').read_text().strip()}
url='http://127.0.0.1:5001/agent/mission/'+args.command
r=requests.get(url,headers=headers,timeout=5) if args.command in ('candidates','status','report') else requests.post(url,headers=headers,json=data,timeout=5)
print(json.dumps(r.json(),indent=2));raise SystemExit(0 if r.ok else 1)
