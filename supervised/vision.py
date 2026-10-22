"""Local Mac perception worker. Started and terminated with an explicit session."""
import base64
import io
import hashlib
import json
import os
from pathlib import Path
import time


def run(base_url, key, session, directory, model_path, yolo_path):
    import cv2
    import numpy as np
    import requests
    from PIL import Image
    from camera import atomic_json
    cv2.setNumThreads(2)
    directory = Path(directory)
    headers = {'X-Robot-Key': key}
    exploration = None
    health = requests.get(base_url + '/health', headers=headers, timeout=2).json()
    if health.get('mode') == 'explore':
        from explore_vision import ExploreVision
        manifest = json.loads((Path(model_path).parents[1] / 'explore-manifest.json').read_text())
        if manifest != health['explore_profile']['models']:
            raise RuntimeError('Installed models differ from validated profile')
        from model_assets import verify_models
        verify_models(Path(model_path).parents[1], manifest)
        exploration = ExploreVision(health['explore_profile'], Path(model_path).parents[1] / 'segformer-b0')
    from supplementary_vision import SupplementaryVision
    supplementary = SupplementaryVision(model_path, yolo_path)
    phase = 'load' 
    last_frame = None
    while True:
        started = time.monotonic()
        try:
            phase = 'capture'
            response = requests.get(base_url + '/observe', headers=headers, timeout=.5)
            response.raise_for_status()
            observation = response.json()
            if observation['session'] != session:
                return
            if observation['frame_id'] == last_frame:
                time.sleep(.04)
                continue
            jpeg = base64.b64decode(observation.pop('jpeg'))
            image = Image.open(io.BytesIO(jpeg)).convert('RGB')
            rgb = np.asarray(image)
            geometry = None
            if exploration:
                phase = 'floor-and-tracking'
                geometry, annotated = exploration.process(rgb, observation)
            phase = 'supplementary-vetoes'
            cues, prediction = supplementary.process(rgb)
            signature = np.asarray(image.convert('L').resize((8, 6))).reshape(-1).tolist()
            payload = {k: observation[k] for k in ('frame_id', 'captured', 'session')}
            payload.update({k:cues[k] for k in ('depth_ready','near_score','hazard')}, error=None, signature=signature)
            if geometry is not None:
                payload['geometry'] = {k:geometry[k] for k in ('configuration_id','tracking','error')}
            elapsed = time.monotonic() - started
            phase = 'publish'
            response = requests.post(base_url + '/perception', headers=headers, json=payload, timeout=.5)
            if not response.ok:
                raise RuntimeError(response.json().get('error', 'Perception rejected'))
            last_frame = observation['frame_id']
            if geometry is not None:
                valid_nav, nav_jpeg = cv2.imencode('.jpg', cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
                atomic_json(directory / 'navigation.json', {**geometry,
                    'supplementary':cues,'frame_id':last_frame,'session':session,'captured':observation['captured'],
                    'received_mac':time.monotonic(),'latency_s':time.monotonic()-started,
                    'jpeg':base64.b64encode(jpeg).decode(),
                    'annotated_jpeg':base64.b64encode(nav_jpeg.tobytes()).decode() if valid_nav else None})
            scaled = cv2.normalize(prediction, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
            coloured = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            coloured = cv2.resize(coloured, image.size)
            cv2.rectangle(coloured, (int(image.width*.25), int(image.height*.35)),
                          (int(image.width*.75), int(image.height*.8)), (255,255,255), 2)
            valid, encoded = cv2.imencode('.jpg', coloured)
            if valid:
                temporary = directory / 'depth.tmp'
                temporary.write_bytes(encoded.tobytes())
                os.replace(temporary, directory / 'depth.jpg')
            temporary = directory / 'frame.tmp'
            temporary.write_bytes(jpeg)
            os.replace(temporary, directory / 'frame.jpg')
            atomic_json(directory / 'vision.json', {**payload, 'latency_s': elapsed,
                        'mac_updated': time.monotonic(), 'camera': observation,
                        'rgb_sha256': hashlib.sha256(jpeg).hexdigest(),
                        'depth_sha256': hashlib.sha256(encoded.tobytes()).hexdigest() if valid else None,
                        'output_range': [float(prediction.min()), float(prediction.max())]})
        except Exception as error:
            if exploration:
                atomic_json(directory / 'navigation.json', {'error':str(error),'tracking':'lost','received_mac':time.monotonic()})
            atomic_json(directory / 'vision.json', {'error': str(error), 'phase': phase,
                        'latency_s': time.monotonic() - started, 'mac_updated': time.monotonic()})
            time.sleep(.2)
        # No inference backlog: discard old frames, fetch newest on the next iteration.
        time.sleep(max(0, .12 - (time.monotonic() - started)))
