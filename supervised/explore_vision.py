"""Local floor perception. Heavy dependencies load only in an explicit session."""
from pathlib import Path
import cv2
import numpy as np
from navigation import FloorTracker, LocalMap, project
from explore_profile import camera_matches, validate_profile


class FloorSegmenter:
    def __init__(self, model_path):
        import torch
        from transformers import AutoImageProcessor, SegformerForSemanticSegmentation
        self.torch=torch
        self.device='mps' if torch.backends.mps.is_available() else 'cpu'
        self.processor=AutoImageProcessor.from_pretrained(str(model_path),local_files_only=True,use_fast=False)
        self.model=SegformerForSemanticSegmentation.from_pretrained(str(model_path),local_files_only=True).to(self.device).eval()
        self.floor_ids=[int(k) for k,v in self.model.config.id2label.items() if v.lower()=='floor']
        if not self.floor_ids:raise ValueError('Floor label missing from model')

    def predict(self,rgb,probability):
        t=self.torch
        with t.inference_mode():
            inputs={k:v.to(self.device) for k,v in self.processor(images=rgb,return_tensors='pt').items()}
            logits=self.model(**inputs).logits
            logits=t.nn.functional.interpolate(logits,size=rgb.shape[:2],mode='bilinear',align_corners=False)
            confidence=logits.softmax(dim=1)[0,self.floor_ids].sum(0).cpu().numpy()
        return (confidence>=probability).astype('uint8')


class ExploreVision:
    def __init__(self,p,model_path,ready=True):
        self.p=validate_profile(p,ready=ready)
        self.segmenter=FloorSegmenter(model_path)
        self.tracker=FloorTracker(p);self.map=LocalMap(p);self.seeded=None

    def process(self,rgb,observation):
        p=self.p;c=p['camera']
        if not camera_matches(p,observation):raise ValueError('Camera profile mismatch')
        rectified=cv2.undistort(rgb,np.array(c['matrix']),np.array(c['distortion']))
        floor=self.segmenter.predict(rectified,p['floor_probability'])
        # Erode uncertainty at object boundaries before projecting any floor.
        floor=cv2.erode(floor,np.ones((5,5),np.uint8))
        gray=cv2.cvtColor(rectified,cv2.COLOR_RGB2GRAY)
        quality = observation.get('quality_reason')
        if observation.get('sharpness',0)<p['min_sharpness'] or observation.get('identical_frames',0)>=3:
            quality = quality or 'Blurred or repeated image'
        if quality and self.tracker.ever_good:
            self.tracker.requires_relocalisation=True
        tracking=self.tracker.update(gray,floor,observation)
        if quality:
            tracking.update(tracking='lost',error=quality)
        if tracking['tracking']!='good':self.map=LocalMap(p)
        m=observation.get('mission') or {}
        if m and m['id']!=self.seeded:
            self.map.seed(m['starting_clearance_m'],observation['captured'],tracking['pose'])
            self.seeded=m['id']
        if tracking['tracking']=='good':self.map.update(floor,tracking['pose'],observation['captured'])
        candidates=self.map.candidates(tracking['pose'],observation['captured'],tracking['uncertainty_m']) if tracking['tracking']=='good' else []
        image=rectified.copy()
        tint=image.copy();tint[floor>0]=(20,180,20);image=cv2.addWeighted(image,.8,tint,.2,0)
        for i,candidate in enumerate(candidates):
            if candidate['action']=='forward':
                uv=project([[candidate['value'],0]],np.linalg.inv(np.array(c['image_to_ground'])))[0]
                at=tuple(np.clip(uv,[0,20],[image.shape[1]-1,image.shape[0]-1]).astype(int))
            else:at=(20 if candidate['action']=='left' else image.shape[1]-170,60)
            cv2.putText(image,f"{i+1}: {candidate['action']} {candidate['value']}",at,cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,0),2)
        return {'configuration_id':p['configuration_id'],'sensor':'usb-camera','models':p['models'],
                **tracking,'map':self.map.snapshot(),'candidates':candidates},image
