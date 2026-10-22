#!/usr/bin/env python3
import json
from pathlib import Path
import statistics
import time
import coremltools as ct
import numpy as np
from PIL import Image
from ultralytics import YOLO

root = Path(__file__).resolve().parents[1]
image = Image.open(root / 'records/stationary-session.jpg').convert('RGB')
depth = ct.models.MLModel(str(root / 'models/depth-v2/DepthAnythingV2SmallF16.mlpackage'))
yolo = YOLO(str(root / 'models/yolo26n.pt'))
results = {}
for name, function in (
    ('depth_coreml', lambda: depth.predict({'image': image.resize((518,392))})),
    ('yolo_mps', lambda: yolo.predict(np.asarray(image)[:,:,::-1].copy(), device='mps', verbose=False)),
    ('yolo_cpu', lambda: yolo.predict(np.asarray(image)[:,:,::-1].copy(), device='cpu', verbose=False))):
    timings = []
    for index in range(8):
        start = time.monotonic()
        function()
        timings.append(time.monotonic() - start)
    results[name] = {'cold_s': timings[0], 'median_s': statistics.median(timings[2:]),
                     'max_warm_s': max(timings[2:]), 'samples_s': timings}
(root / 'records/mac-vision-benchmark.json').write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
