#!/usr/bin/env python3
"""Download pinned floor-model assets only; does not start camera or robot."""
import sys
import argparse
import json
from pathlib import Path
from huggingface_hub import HfApi, snapshot_download
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'supervised'))
from model_assets import sha
repo='nvidia/segformer-b0-finetuned-ade-512-512'
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--revision',default='489d5cd81a0b59fab9b7ea758d3548ebe99677da');args=parser.parse_args()
api=HfApi();info=api.model_info(repo,revision=args.revision);revision=info.sha
names={s.rfilename for s in info.siblings}
weights='model.safetensors' if 'model.safetensors' in names else 'pytorch_model.bin'
path=ROOT/'models/segformer-b0'
snapshot_download(repo,revision=revision,local_dir=path,allow_patterns=['config.json','preprocessor_config.json',weights])
manifest={'floor':{'repository':repo,'revision':revision,'sha256':sha(path/weights),
          'assets':{'segformer-b0/'+name:sha(path/name) for name in ('config.json','preprocessor_config.json',weights)}},
          'depth':{'path':'depth-v2/DepthAnythingV2SmallF16.mlpackage','sha256':sha(ROOT/'models/depth-v2/DepthAnythingV2SmallF16.mlpackage')},
          'detector':{'path':'yolo26n.pt','sha256':sha(ROOT/'models/yolo26n.pt')}}
(ROOT/'models/explore-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest,indent=2))
