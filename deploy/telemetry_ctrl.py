"""Read chassis telemetry; all movement/peripheral methods remain disabled.

The only serial command this module can send is the fixed T=130 query.
No generic JSON, motor, module selection, calibration or firmware commands.
"""
import json
import math
import threading
import time
import serial
from stationary_ctrl import BaseController as DisabledController

class BaseController(DisabledController):
    def __init__(self, port, baud):
        super().__init__()
        self.last_feedback = 0.0
        self._lock = threading.Lock()
        self._port = serial.Serial(port, baud, timeout=0.1,
                                   write_timeout=0.25, exclusive=True)
        threading.Thread(target=self._read, daemon=True,
                         name='chassis-telemetry').start()

    def _read(self):
        next_query = 0.0
        try:
            while True:
                if time.monotonic() >= next_query:
                    self._port.write(b'{"T":130}\n')
                    next_query = time.monotonic() + 0.5
                line = self._port.readline(4096)
                if not line:
                    continue
                try:
                    data = json.loads(line)
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(data, dict) or data.get('T') != 1001:
                    continue
                if any(type(data.get(key)) not in (int, float)
                       or not math.isfinite(data[key]) for key in ('L', 'R', 'v')):
                    continue
                with self._lock:
                    self.base_data = data
                    self.last_feedback = time.monotonic()
        except Exception as exc:
            print(f'Chassis telemetry unavailable: {exc}', flush=True)
            # Camera can remain available; health/voltage become stale/unknown.

    def status(self):
        with self._lock:
            age = time.monotonic() - self.last_feedback
            connected = self.base_data is not None and age < 1.5
            return {'controller_connected': connected, 'motion_enabled': False,
                    'telemetry_age_s': round(age, 3) if self.base_data else None,
                    'battery_voltage_v': self.base_data['v'] / 100 if connected else None,
                    'left_speed_m_s': self.base_data['L'] if connected else None,
                    'right_speed_m_s': self.base_data['R'] if connected else None}

    def feedback_data(self):
        time.sleep(0.1)
        with self._lock:
            return self.base_data if time.monotonic() - self.last_feedback < 1.5 else None

    def voltage_raw(self):
        with self._lock:
            return self.base_data['v'] if self.base_data and time.monotonic() - self.last_feedback < 1.5 else None
