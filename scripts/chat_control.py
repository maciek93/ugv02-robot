#!/usr/bin/env python3
"""Trusted chat-to-operator bridge. Never invent a human movement approval.

Approval requires the exact proposal ID and a recorded human instruction from
this conversation. This records provenance, not cryptographic chat attestation.
"""
import argparse
import json
from pathlib import Path
import time
import requests

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
commands = parser.add_subparsers(dest='operation', required=True)
session = commands.add_parser('session')
session.add_argument('action', choices=('start','pause','resume','end'))
session.add_argument('--mode', choices=('bench','calibration','tour','explore','manual','mapping'), default='bench')
approve = commands.add_parser('approve')
approve.add_argument('proposal_id')
approve.add_argument('--human-instruction', required=True)
profile = commands.add_parser('profile')
profile.add_argument('file', type=Path)
profile.add_argument('--human-instruction', required=True)
mission = commands.add_parser('approve-mission')
mission.add_argument('mission_id')
mission.add_argument('--human-instruction', required=True)
explore = commands.add_parser('explore-profile')
explore.add_argument('file', type=Path)
explore.add_argument('--human-instruction', required=True)
metric_profile = commands.add_parser('metric-profile')
metric_profile.add_argument('file', type=Path)
metric_profile.add_argument('--human-instruction', required=True)
metric_mission = commands.add_parser('approve-metric-mission')
metric_mission.add_argument('mission_id')
metric_mission.add_argument('--human-instruction', required=True)
args = parser.parse_args()
keys = json.loads((root / '.runtime/pi-keys.json').read_text())

def pi(path, body):
    r = requests.post('http://127.0.0.1:5002' + path,
                      headers={'X-Robot-Key':keys['operator']},json=body,timeout=3)
    r.raise_for_status()
    return r.json()

def record(**details):
    with (root / 'records/supervised-actions.jsonl').open('a') as log:
        log.write(json.dumps({'time':time.strftime('%Y-%m-%dT%H:%M:%S%z'),**details})+'\n')

if args.operation == 'session':
    agent_key = (root / '.runtime/agent-key').read_text().strip()
    r = requests.post('http://127.0.0.1:5001/agent/session',
                     headers={'X-Agent-Key':agent_key},
                     json={'operation':args.action,'mode':args.mode}, timeout=5)
    r.raise_for_status()
    result = r.json()
elif args.operation == 'approve':
    state = requests.get('http://127.0.0.1:5002/health',headers={'X-Robot-Key':keys['operator']},timeout=2).json()
    proposal = state.get('proposal')
    if not proposal or proposal['id'] != args.proposal_id:
        raise SystemExit('Proposal changed; obtain approval for the current proposal.')
    record(event='chat_human_approval', proposal=proposal, human_instruction=args.human_instruction)
    result = pi('/operator/approve', {'proposal_id':args.proposal_id,'clearance':True})
elif args.operation == 'approve-mission':
    state = requests.get('http://127.0.0.1:5002/health',headers={'X-Robot-Key':keys['operator']},timeout=2).json()
    proposal = state['mission']['proposal']
    if not proposal or proposal['id'] != args.mission_id:
        raise SystemExit('Mission proposal changed; review the new bounded mission.')
    record(event='chat_mission_approval', proposal=proposal, human_instruction=args.human_instruction)
    result = pi('/operator/mission/approve', {'mission_id':args.mission_id})
elif args.operation == 'approve-metric-mission':
    state = requests.get('http://127.0.0.1:5002/health',headers={'X-Robot-Key':keys['operator']},timeout=2).json()
    proposal = state['metric_mission']['proposal']
    if not proposal or proposal['id'] != args.mission_id:
        raise SystemExit('Metric mission proposal changed; review the new bounded mission.')
    record(event='chat_metric_mission_approval', proposal=proposal, human_instruction=args.human_instruction)
    result = pi('/operator/metric/mission/approve', {'mission_id':args.mission_id})
elif args.operation == 'explore-profile':
    import sys
    sys.path.insert(0,str(root/'supervised'))
    from explore_profile import validate_profile, verify_recorded_validation
    calibration=json.loads(args.file.read_text())
    validate_profile(calibration)
    verify_recorded_validation(calibration,root)
    record(event='chat_explore_profile_review', profile=calibration, human_instruction=args.human_instruction)
    result=pi('/operator/explore-profile',calibration)
elif args.operation == 'metric-profile':
    import sys
    sys.path.insert(0,str(root/'supervised'))
    from metric_mission import validate_metric_profile
    calibration=json.loads(args.file.read_text())
    validate_metric_profile(calibration)
    record(event='chat_metric_profile_review', profile=calibration, human_instruction=args.human_instruction)
    result=pi('/operator/metric-profile',calibration)
else:
    calibration = json.loads(args.file.read_text())
    record(event='chat_calibration_review', profile=calibration, human_instruction=args.human_instruction)
    result = pi('/operator/profile', calibration)
print(json.dumps(result,indent=2))
