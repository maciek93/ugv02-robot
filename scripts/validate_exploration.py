#!/usr/bin/env python3
"""Offline replay and physical acceptance audit. Never connects to robot services."""
import argparse
import json
from pathlib import Path
import sys
import time
import math
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'supervised'))
from camera import atomic_json
from explore_profile import configuration_fingerprint


def replay(profile,dataset,output):
    import cv2
    from explore_vision import ExploreVision
    from navigation import LocalMap
    from model_assets import verify_models
    from supplementary_vision import SupplementaryVision, veto_reason
    spec=json.loads(dataset.read_text());p=json.loads(profile.read_text())
    if spec.get('split')!='held-out':raise ValueError('Validation must use a held-out dataset')
    verify_models(ROOT/'models',p['models'])
    supplementary=SupplementaryVision(ROOT/'models/depth-v2/DepthAnythingV2SmallF16.mlpackage',ROOT/'models/yolo26n.pt')
    rows=[];latencies=[]
    for sequence in spec['sequences']:
        engine=ExploreVision(p,ROOT/'models/segformer-b0',ready=False)
        for sample in sequence['frames']:
            rgb=cv2.cvtColor(cv2.imread(str(dataset.parent/sample['image'])),cv2.COLOR_BGR2RGB)
            observation=json.loads((dataset.parent/sample['observation']).read_text())
            started=time.monotonic()
            try:
                n,_=engine.process(rgb,observation)
                clear=n['tracking']=='good' and LocalMap.restore(p,n['map']).clear(n['pose'],sample.get('action','forward'),sample.get('value',.1),observation['captured'],n['uncertainty_m'])
                cues,_=supplementary.process(rgb)
                reason=n['error'] or veto_reason(p,cues)
                clear=clear and reason is None
            except Exception as e:clear=False;reason=str(e)
            elapsed=time.monotonic()-started;latencies.append(elapsed)
            if sample.get('expected') not in ('clear','blocked','warmup'):raise ValueError('Every frame needs a human label')
            rows.append({'image':sample['image'],'expected':sample['expected'],'clear':bool(clear),'reason':reason,'latency_s':elapsed,'category':sequence['category']})
    scored=[r for r in rows if r['expected']!='warmup'];categories={r['category'] for r in scored}
    required={'furniture','chair-leg','doorway','reflective','clear-floor'}
    false_clear=[r for r in scored if r['expected']=='blocked' and r['clear']]
    false_block=[r for r in scored if r['expected']=='clear' and not r['clear']]
    passed=bool(scored) and required<=categories and not false_clear and not false_block
    report={'kind':'held-out-perception','profile_fingerprint':configuration_fingerprint(p),'passed':passed,'dataset':str(dataset.resolve()),'models':p['models'],'configuration_id':p['configuration_id'],'false_clear':len(false_clear),'false_block':len(false_block),
            'missing_categories':sorted(required-categories),'max_local_processing_s':max(latencies,default=None),
            'latency_validation_passed':False,'note':'Replay processing time excludes network/capture and is not live latency validation.','frames':rows}
    atomic_json(output,report);print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2));return passed


def acceptance(path,output):
    manifest=json.loads(path.read_text());checks={}
    trials=manifest.get('trials',[])
    for kind,targets in [('forward',[.1,.25,.5]),('left',[15]),('right',[15])]:
        for target in targets:
            rows=[r for r in trials if r.get('action')==kind and r.get('target')==target]
            tolerance=max(.03,.2*target) if kind=='forward' else 5
            checks[f'{kind}_{target}']=len(rows)>=3 and all(type(r.get('measured')) in (int,float) and math.isfinite(r['measured']) and abs(r['measured']-target)<=tolerance and r.get('stopped') is True and r.get('direction_correct') is True and (path.parent/r.get('evidence','')).is_file() for r in rows)
    for name in ('camera_loss','network_loss','viewer_loss','obstacle_stop','esp32_command_loss','shutdown','restart_disarmed'):
        r=manifest.get(name,{})
        checks[name]=r.get('passed') is True and (path.parent/r.get('evidence','')).is_file()
        if name in ('camera_loss','network_loss','viewer_loss','obstacle_stop'):
            measured, bound=r.get('measured_stop_m'),r.get('envelope_m')
            checks[name] = checks[name] and all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in (measured,bound)) and measured<=bound
        if name=='esp32_command_loss':
            measured,bound=r.get('measured_timeout_s'),r.get('verified_upper_timeout_s')
            checks[name] = checks[name] and all(type(v) in (int,float) and math.isfinite(v) and v>0 for v in (measured,bound)) and measured<=bound
    surveys=manifest.get('surveys',[])
    checks['two_room_surveys']=len(surveys)>=2 and all(r.get('two_rooms') is True and r.get('no_contact') is True and r.get('recognised_finish') is True and (path.parent/r.get('evidence','')).is_file() for r in surveys)
    result={'passed':all(checks.values()),'checks':checks,'source':str(path.resolve()),'note':'Audits recorded human evidence, not independent attestation.'}
    atomic_json(output,result);print(json.dumps(result,indent=2));return result['passed']

p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
a=s.add_parser('replay');a.add_argument('profile',type=Path);a.add_argument('dataset',type=Path);a.add_argument('--output',type=Path,required=True)
a=s.add_parser('acceptance');a.add_argument('manifest',type=Path);a.add_argument('--output',type=Path,required=True)
a=p.parse_args()
passed=replay(a.profile,a.dataset,a.output) if a.command=='replay' else acceptance(a.manifest,a.output)
sys.exit(0 if passed else 1)
