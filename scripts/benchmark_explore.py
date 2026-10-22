#!/usr/bin/env python3
"""Benchmark all three models on saved RGB, without camera or motor access."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
import cv2
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'supervised'))
p=argparse.ArgumentParser(description=__doc__);p.add_argument('image',type=Path);p.add_argument('--samples',type=int,default=20);p.add_argument('--output',type=Path,default=ROOT/'records/explore-model-benchmark.json');a=p.parse_args()
from explore_vision import FloorSegmenter
import coremltools as ct
from ultralytics import YOLO
floor=FloorSegmenter(ROOT/'models/segformer-b0')
depth=ct.models.MLModel(str(ROOT/'models/depth-v2/DepthAnythingV2SmallF16.mlpackage'),compute_units=ct.ComputeUnit.ALL)
detector=YOLO(str(ROOT/'models/yolo26n.pt'))
spec=depth.get_spec().description;feature=spec.input[0];output=spec.output[0].name
rgb=np.asarray(Image.open(a.image).convert('RGB'));im=Image.fromarray(rgb).resize((feature.type.imageType.width,feature.type.imageType.height))
rows=[]
for i in range(a.samples+1):
    start=time.monotonic();mask=floor.predict(rgb,.95);f=time.monotonic()
    d=np.asarray(depth.predict({feature.name:im})[output]);dt=time.monotonic()
    boxes=detector.predict(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),device='mps',imgsz=640,conf=.25,verbose=False)[0].boxes
    end=time.monotonic();rows.append({'floor_s':f-start,'depth_s':dt-f,'detector_s':end-dt,'total_s':end-start})
report={'kind':'saved-image-three-model-benchmark','image':str(a.image.resolve()),'models':json.loads((ROOT/'models/explore-manifest.json').read_text()),
        'backend':floor.device,'cold':rows[0],'warm':rows[1:],'warm_max_s':max(r['total_s'] for r in rows[1:]),
        'warm_median_s':float(np.median([r['total_s'] for r in rows[1:]])),
        'live_latency_validated':False,'limitations':'Excludes capture, network, geometric tracking, mapping and motor delivery. Not permission to drive.'}
a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('warm','models')},indent=2))
