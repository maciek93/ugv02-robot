#!/usr/bin/env python3
"""Attach human measurements to one completed physical trial; report rates."""
import argparse
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('experiment',type=Path)
parser.add_argument('--travel-mm',type=float)
parser.add_argument('--angle-deg',type=float)
parser.add_argument('--braking-mm',type=float)
parser.add_argument('--notes',required=True)
args=parser.parse_args()
directory=args.experiment.resolve()
if not directory.is_relative_to(root/'data/experiments'):
    raise SystemExit('Measurements must belong to a project experiment')
trial=json.loads((directory/'trial.json').read_text())
if trial['physical_context']!='clear-floor' or trial['status']!='completed':
    raise SystemExit('Only completed floor trials can calibrate physical movement')
import math
for field in ('travel_mm','angle_deg','braking_mm'):
    value=getattr(args,field)
    if value is not None and (not math.isfinite(value) or value<0):
        raise SystemExit('Measurements must be finite and nonnegative')
measurement={'travel_mm':args.travel_mm,'angle_deg':args.angle_deg,
             'braking_mm':args.braking_mm,'human_notes':args.notes,
             'action':trial['action'],'requested_motor_s':trial['value'],
             'configuration':trial.get('configuration')}
(directory/'measurement.json').write_text(json.dumps(measurement,indent=2))
trial['physical_result']=args.notes
(directory/'trial.json').write_text(json.dumps(trial,indent=2))
rows=[]
for path in (root/'data/experiments').glob('*/measurement.json'):
    row=json.loads(path.read_text())
    if row['action'].startswith('pulse_') and row.get('configuration')==measurement['configuration']:
        rows.append(row)
forward=[r for r in rows if r['action']=='pulse_forward' and r['travel_mm'] is not None]
left=[r for r in rows if r['action']=='pulse_left' and r['angle_deg'] is not None]
right=[r for r in rows if r['action']=='pulse_right' and r['angle_deg'] is not None]
report={'configuration':measurement['configuration'], 'measured_forward_trials':len(forward),'measured_left_trials':len(left),
        'measured_right_trials':len(right),'depth_validated':False,
        'upper_speed_m_s':max((r['travel_mm']/1000/r['requested_motor_s'] for r in forward),default=0)*1.25,
        'upper_turn_deg_s':max((r['angle_deg']/r['requested_motor_s'] for r in left+right),default=0)*1.25,
        'max_braking_mm':max((r['braking_mm'] or 0 for r in rows),default=0)}
(root/'data/movement-calibration-draft.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
