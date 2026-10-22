"""Session-owned UVC capture. No camera is opened on import."""
import json
import os
import signal
import time
import threading
from pathlib import Path


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False))
    os.replace(temporary, path)


def usb_identity(device, sysfs='/sys/class/video4linux'):
    """Bind calibration to USB product and serial, or its physical USB port."""
    name = 'video' + str(device) if type(device) is int else Path(device).resolve().name
    node = Path(sysfs) / name / 'device'
    if not node.exists():
        return None
    node = node.resolve()
    for parent in (node, *node.parents):
        if (parent / 'idVendor').is_file() and (parent / 'idProduct').is_file():
            vendor = (parent / 'idVendor').read_text().strip()
            product = (parent / 'idProduct').read_text().strip()
            serial = (parent / 'serial').read_text().strip() if (parent / 'serial').is_file() else None
            return ':'.join((vendor, product, 'serial=' + serial if serial else 'port=' + parent.name))
    return None


def capture(directory, device, epoch, rotation):
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    directory = Path(directory)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 15)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    focus_locked = bool(camera.set(cv2.CAP_PROP_AUTOFOCUS, 0))
    initial_focus = camera.get(cv2.CAP_PROP_FOCUS)
    identity = usb_identity(device)
    previous = None
    identical_frames = 0
    sequence = 0
    try:
        if not camera.isOpened():
            raise RuntimeError('UVC device did not open')
        while not stopping:
            ok, frame = camera.read()
            captured = time.monotonic()
            if not ok:
                raise RuntimeError('UVC read failed')
            if rotation == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            small = cv2.resize(gray, (160, 120))
            flow = None
            if previous is not None:
                identical_frames = identical_frames + 1 if np.array_equal(previous, small) else 0
                vectors = cv2.calcOpticalFlowFarneback(previous, small, None, .5, 2, 9, 2, 5, 1.1, 0)
                flow = float(np.median(np.linalg.norm(vectors, axis=2)))
            previous = small
            sequence += 1
            frame_id = f'{epoch}-{sequence}'
            target = directory / (frame_id + '.jpg')
            valid, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not valid:
                raise RuntimeError('JPEG encode failed')
            target.write_bytes(jpeg.tobytes())
            brightness = float(gray.mean())
            contrast = float(gray.std())
            reasons = []
            if brightness < 20 or brightness > 240:
                reasons.append('Image too dark or washed out')
            if contrast < 8:
                reasons.append('Image has too little detail')
            metadata = {
                'frame_id': frame_id, 'captured': captured, 'width': frame.shape[1], 'height': frame.shape[0],
                'rotation_degrees': rotation, 'device': device, 'identity': identity,
                'focus': float(camera.get(cv2.CAP_PROP_FOCUS)),
                'focus_stable': bool(focus_locked and camera.get(cv2.CAP_PROP_AUTOFOCUS) == 0 and camera.get(cv2.CAP_PROP_FOCUS) == initial_focus),
                'brightness': brightness, 'contrast': contrast,
                'sharpness': float(cv2.Laplacian(gray, cv2.CV_64F).var()),
                'flow_pixels': flow, 'identical_frames': identical_frames,
                'quality_reason': '; '.join(reasons) or None}
            atomic_json(directory / (frame_id + '.json'), metadata)
            atomic_json(directory / 'camera.json', metadata)
            old = directory / f'{epoch}-{sequence - 8}.jpg'
            old.unlink(missing_ok=True)
            old.with_suffix('.json').unlink(missing_ok=True)
    except Exception as error:
        atomic_json(directory / 'camera.json', {'error': str(error), 'captured': 0})
    finally:
        camera.release()


class Camera:
    def __init__(self, directory, device, rotation=0):
        if type(rotation) is not int or rotation not in (0, 180):
            raise ValueError('Camera rotation must be 0 or 180 degrees')
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.device = device
        self.rotation = rotation
        self.process = None
        self.lock = threading.RLock()

    def start(self):
        import multiprocessing
        import secrets
        with self.lock:
            self.close()
            self.process = multiprocessing.get_context('spawn').Process(
                target=capture, args=(str(self.directory), self.device, secrets.token_hex(4), self.rotation))
            self.process.start()

    def read(self):
        try:
            metadata = json.loads((self.directory / 'camera.json').read_text())
            if not self.process or not self.process.is_alive():
                return {'error': 'Camera worker is off'}
            if metadata.get('error'):
                return metadata
            metadata['age_s'] = time.monotonic() - metadata['captured']
            if metadata['age_s'] > .5:
                return {'error': 'Camera frame stale'}
            return metadata
        except (OSError, ValueError, KeyError):
            return {'error': 'Camera warming up'}

    def close(self):
        with self.lock:
            if self.process:
                self.process.terminate()
                self.process.join(1.5)
                if self.process.is_alive():
                    self.process.kill()
                    self.process.join(.5)
                self.process = None
            for path in self.directory.glob('*'):
                path.unlink(missing_ok=True)
