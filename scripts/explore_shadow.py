#!/usr/bin/env python3
"""Capture stationary three-model replay data and Pi-clock latency; no driving."""
import argparse
import base64
from collections import deque
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'supervised'))
p=argparse.ArgumentParser(description=__doc__);p.add_argument('profile',type=Path);p.add_argument('directory',type=Path);p.add_argument('--frames',type=int,default=100);a=p.parse_args()
import cv2
import numpy as np
import requests
from PIL import Image
from explore_vision import ExploreVision
from camera import atomic_json
from explore_profile import configuration_fingerprint
from model_assets import verify_models
from supplementary_vision import SupplementaryVision
profile=json.loads(a.profile.read_text());verify_models(ROOT/'models',profile['models'])
engine=ExploreVision(profile,ROOT/'models/segformer-b0',ready=False)
supplementary=SupplementaryVision(ROOT/'models/depth-v2/DepthAnythingV2SmallF16.mlpackage',ROOT/'models/yolo26n.pt')
keys=json.loads((ROOT/'.runtime/pi-keys.json').read_text());headers={'X-Robot-Key':keys['navigation']};base='http://127.0.0.1:5002'
a.directory.mkdir(parents=True,exist_ok=True)
rows=[];seen=set();deadline=time.monotonic()+max(60,a.frames)
while len(rows)<a.frames and time.monotonic()<deadline:
    h=requests.get(base+'/health',headers=headers,timeout=2);h.raise_for_status();health=h.json()
    if health['mode']!='calibration' or health['moving'] or health['execution']:raise SystemExit('Stationary calibration session required. No motion was requested.')
    response=requests.get(base+'/observe',headers=headers,timeout=2);response.raise_for_status();o=response.json()
    if o['frame_id'] in seen:time.sleep(.02);continue
    seen.add(o['frame_id']);jpeg=base64.b64decode(o.pop('jpeg'));rgb=cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
    start=time.monotonic();geometry,annotated=engine.process(rgb,o)
    cues,prediction=supplementary.process(rgb)
    processing=time.monotonic()-start
    probe=requests.post(base+'/navigation/probe',headers=headers,json={k:o[k] for k in ('session','frame_id','captured')},timeout=.5)
    latency=probe.json().get('capture_to_probe_s') if probe.ok else None
    name=f'{len(rows):04}';(a.directory/(name+'.jpg')).write_bytes(jpeg);atomic_json(a.directory/(name+'.json'),o)
    cv2.imwrite(str(a.directory/(name+'.annotated.jpg')),cv2.cvtColor(annotated,cv2.COLOR_RGB2BGR))
    atomic_json(a.directory/(name+'.geometry.json'),geometry)
    rows.append({'image':name+'.jpg','observation':name+'.json','expected':'UNLABELLED','processing_s':processing,
                 'capture_to_probe_s':latency,'tracking':geometry['tracking'],'depth_range':[float(prediction.min()),float(prediction.max())],
                 'supplementary':cues})
    print(f'{len(rows)}/{a.frames}: capture-to-probe={latency}, tracking={geometry["tracking"]}',flush=True)
# Warm-up is recorded separately and never silently accepted as movement-ready.
warm=rows[5:];passed=len(warm)>=95 and all(r['capture_to_probe_s'] is not None and r['capture_to_probe_s']<.35 for r in warm)
atomic_json(a.directory/'latency.json',{'kind':'stationary-navigation-latency','profile_fingerprint':configuration_fingerprint(profile),'passed':passed,'samples':len(warm),'warmup_frames':5,'measurement':'Pi capture to stationary navigation probe, all models + geometry',
    'motion_delivery_validated':False,'rows':rows,'models':profile['models']})
atomic_json(a.directory/'dataset-draft.json',{'split':'UNLABELLED','sequences':[{'category':'UNLABELLED','frames':rows}]})
print('Saved stationary data. Label and reserve held-out scenes separately. This command does not validate physical stopping or motor delivery.')
