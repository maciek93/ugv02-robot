"""One camera reader, shared live frames, and bounded observation freshness."""
import logging
import os
import threading
import time

class CameraStream:
    def __init__(self, read_frame, on_failure=None):
        self.read_frame = read_frame
        self.on_failure = on_failure
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0
        self.updated = 0.0
        threading.Thread(target=self.run, daemon=True, name='camera-reader').start()

    def run(self):
        try:
            while True:
                frame = self.read_frame()
                if not isinstance(frame, bytes) or not frame.startswith(b'\xff\xd8'):
                    raise RuntimeError('Camera did not produce a JPEG')
                with self.condition:
                    self.frame = frame
                    self.sequence += 1
                    self.updated = time.monotonic()
                    self.condition.notify_all()
        except Exception:
            logging.exception('Camera reader failed; systemd will restart the app')
            if self.on_failure:
                try:
                    self.on_failure()
                except Exception:
                    logging.exception('Stop on camera failure also failed')
            os._exit(1)

    def snapshot(self):
        with self.condition:
            if self.frame is None or time.monotonic() - self.updated > 1.0:
                return None
            return self.frame

    def frames(self):
        sequence = -1
        while True:
            with self.condition:
                if not self.condition.wait_for(lambda: self.sequence != sequence, timeout=1.0):
                    return
                frame = self.frame
                sequence = self.sequence
                if frame is None:
                    continue
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
