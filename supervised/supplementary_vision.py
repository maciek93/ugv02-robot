"""Identical USB-camera veto inference for live, shadow and held-out replay."""
import math


def veto_reason(profile, observation):
    if observation.get('hazard'):
        return observation['hazard']
    score = observation.get('near_score')
    if (not observation.get('depth_ready') or type(score) not in (int, float)
            or not math.isfinite(score) or not profile['score_low'] <= score < profile['near_limit']):
        return 'Relative-depth veto or unvalidated score'
    return None


class SupplementaryVision:
    def __init__(self, depth_path, detector_path):
        import coremltools as ct
        from ultralytics import YOLO
        self.depth = ct.models.MLModel(str(depth_path), compute_units=ct.ComputeUnit.ALL)
        self.detector = YOLO(str(detector_path))
        spec = self.depth.get_spec().description
        self.feature, self.output = spec.input[0], spec.output[0].name

    def process(self, rgb):
        import cv2
        import numpy as np
        from PIL import Image
        f = self.feature
        image = Image.fromarray(rgb).resize((f.type.imageType.width, f.type.imageType.height))
        prediction = np.asarray(self.depth.predict({f.name: image})[self.output]).squeeze()
        if prediction.ndim != 2 or not np.isfinite(prediction).all():
            raise RuntimeError('Depth model returned an invalid map')
        boxes = self.detector.predict(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), verbose=False,
                                      device='mps', imgsz=640, conf=.25)[0].boxes
        labels = [self.detector.names[int(k)] for k in boxes.cls.cpu().tolist()]
        hazards = sorted(set(labels) & {'person', 'dog', 'cat'})
        h, w = prediction.shape
        corridor = prediction[int(h*.35):int(h*.80), int(w*.25):int(w*.75)]
        return {'depth_ready': True, 'near_score': float(np.percentile(corridor, 95)),
                'hazard': 'Visible ' + ', '.join(hazards) if hazards else None,
                'detected_classes': labels, 'sensor': 'usb-camera'}, prediction
