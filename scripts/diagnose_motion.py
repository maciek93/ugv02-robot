#!/usr/bin/env python3
"""Summarise recorded commands, raw encoders and human motion evidence separately."""
import argparse
import json
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);a=p.parse_args()
t=json.loads((a.trial/'trial.json').read_text())
before=t.get('before',{}).get('chassis',{}).get('data',{})
after=t.get('after',{}).get('chassis',{}).get('data',{})
deltas={k:after[k]-before[k] for k in ('odl','odr') if k in before and k in after}
m=a.trial/'measurement.json';measurement=json.loads(m.read_text()) if m.exists() else None
report={'trial':str(a.trial.resolve()),'command':{'action':t['action'],'requested_value':t['value'],'proposal':t.get('proposal')},
        'raw_encoder_delta':deltas,'human_measurement':measurement or {'travel_mm':t.get('physical_travel_mm'),'notes':t.get('physical_result')},
        'recorded_status':t['status'],'conflict':t.get('measurement_conflict'),
        'calibration_eligible':False,
        'next_checks':['Confirm actual physical direction on supported wheels and floor.',
                       'Determine encoder signs/units and whether values reset across direction changes.',
                       'Measure stopping and commanded versus actual travel; do not infer movement from counts.']}
print(json.dumps(report,indent=2))
