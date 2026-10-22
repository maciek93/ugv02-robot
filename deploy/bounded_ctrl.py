"""Telemetry plus explicit guarded movement API; generic vendor commands blocked."""
import atexit
import json
from pathlib import Path
import signal
import threading
import time
import serial
from telemetry_ctrl import BaseController as TelemetryController
from stationary_ctrl import BaseController as DisabledController
from motion_guard import MotionGuard

class BaseController(TelemetryController):
    ARM_FILE = Path('/run/ugv-web/motion-enabled')

    def __init__(self, port, baud):
        DisabledController.__init__(self)
        self.last_feedback = 0.0
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._closed = False
        self._stop_retry = False
        self.observation_ok = lambda: False
        # Later sensor providers return a rejection reason or None. This remains
        # supervised manual/bench control; autonomous roaming is not exposed.
        self.forward_check = lambda left, right: None
        self._port = serial.Serial(port, baud, timeout=0.1, write_timeout=0.25, exclusive=True)
        self.guard = MotionGuard(self._wheels, self.motion_armed, self._unsafe)
        self.guard.stop('startup')
        threading.Thread(target=self._read, daemon=True, name='chassis-telemetry').start()
        threading.Thread(target=self._watchdog, daemon=True, name='motion-watchdog').start()
        atexit.register(self.close)
        signal.signal(signal.SIGTERM, self._terminate)

    def motion_armed(self):
        try:
            remaining = float(self.ARM_FILE.read_text()) - time.monotonic()
            return 0 < remaining <= 120
        except (OSError, ValueError):
            return False

    def _unsafe(self, left, right):
        with self._lock:
            if self.base_data is None or time.monotonic() - self.last_feedback > 0.75:
                return 'Chassis telemetry missing or stale'
        if not self.observation_ok():
            return 'Camera observation missing or stale'
        return self.forward_check(left, right)

    def _wheels(self, left, right):
        try:
            with self._write_lock:
                payload = (json.dumps({'T': 1, 'L': left, 'R': right}) + '\n').encode()
                if self._port.write(payload) != len(payload):
                    raise RuntimeError('Incomplete motor command write')
        except Exception:
            if left == right == 0:
                self._stop_retry = True
            raise
        if left == right == 0:
            self._stop_retry = False
        print(f'MOTOR monotonic={time.monotonic():.6f} left={left:.3f} right={right:.3f}', flush=True)

    def _read(self):
        # Single reader, no queued commands; fixed telemetry query is independent
        # of movement lease renewal. Queries never extend movement.
        next_query = 0.0
        while not self._closed:
            try:
                if time.monotonic() >= next_query:
                    with self._write_lock:
                        self._port.write(b'{"T":130}\n')
                    next_query = time.monotonic() + 0.5
                line = self._port.readline(4096)
                if line:
                    self._accept_telemetry(line)
            except Exception as exc:
                print(f'Chassis telemetry error: {exc}', flush=True)
                return

    def _accept_telemetry(self, line):
        import math
        try:
            data = json.loads(line)
        except (ValueError, UnicodeError):
            return
        if not isinstance(data, dict) or data.get('T') != 1001:
            return
        if any(type(data.get(key)) not in (int, float) or not math.isfinite(data[key]) for key in ('L', 'R', 'v')):
            return
        with self._lock:
            self.base_data = data
            self.last_feedback = time.monotonic()

    def _watchdog(self):
        while not self._closed:
            try:
                if self._stop_retry:
                    self.guard.stop('retry pending stop')
                self.guard.tick()
            except Exception as exc:
                print(f'Stop transport error: {exc}', flush=True)
                # Retry zero-speed writes if transport recovers. ESP32's own
                # documented 3-second fallback remains the final protection.
                try:
                    self.guard.stop('watchdog transport retry')
                except Exception:
                    pass
            time.sleep(0.02)

    def status(self):
        status = super().status()
        status.update(self.guard.status())
        return status

    def close(self):
        if not self._closed:
            self._closed = True
            try:
                self.guard.stop('service shutdown')
            finally:
                self._port.close()

    def _terminate(self, signum, frame):
        self.close()
        raise SystemExit(0)
