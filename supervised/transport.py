"""Only fixed telemetry and bounded wheel commands; no generic firmware API."""
import json
import logging
import math
import threading
import time


class Chassis:
    def __init__(self, device='/dev/ttyAMA0', motor_polarity=(1, 1)):
        import serial
        if len(motor_polarity) != 2 or any(type(x) is not int or x not in (-1, 1) for x in motor_polarity):
            raise ValueError('Motor polarity must contain two signs')
        self.motor_polarity = tuple(motor_polarity)
        self.port = serial.Serial(device, 115200, timeout=.05, write_timeout=.10, exclusive=True)
        self.write_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.quit = threading.Event()
        self.data = None
        self.updated = 0
        self.error = None
        self.motor_mode = 'speed'
        self.thread = threading.Thread(target=self.read, name='telemetry', daemon=True)
        self.wheels(0, 0)
        self.thread.start()

    def wheels(self, left, right):
        # Physical channel mapping: firmware 'L' is physical Right motor,
        # firmware 'R' is physical Left motor.
        raw_l = right * self.motor_polarity[1]
        raw_r = left * self.motor_polarity[0]
        payload = (json.dumps({'T': 1, 'L': raw_l, 'R': raw_r}, allow_nan=False) + '\n').encode()
        with self.write_lock:
            if left == right == 0 and self.motor_mode == 'pwm':
                pwm_stop = b'{"T":11,"L":0,"R":0}\n'
                if self.port.write(pwm_stop) != len(pwm_stop):
                    raise OSError('Incomplete PWM stop write')
            if self.port.write(payload) != len(payload):
                raise OSError('Incomplete motor write')
            self.motor_mode = 'speed'
        logging.info('MOTOR monotonic=%.6f left=%.3f right=%.3f', time.monotonic(), left, right)

    def pwm(self, left, right):
        if any(type(value) is not int or not -80 <= value <= 80 for value in (left, right)):
            raise ValueError('Manual PWM must be an integer between -80 and 80')
        # Physical channel mapping: firmware 'L' is physical Right motor,
        # firmware 'R' is physical Left motor.
        raw_l = right * self.motor_polarity[1]
        raw_r = left * self.motor_polarity[0]
        payload = (json.dumps({'T': 11, 'L': raw_l, 'R': raw_r}) + '\n').encode()
        with self.write_lock:
            if self.port.write(payload) != len(payload):
                raise OSError('Incomplete PWM write')
            self.motor_mode = 'pwm'
        logging.info('MOTOR_PWM monotonic=%.6f left=%d right=%d', time.monotonic(), left, right)

    def read(self):
        query_at = 0
        while not self.quit.is_set():
            try:
                if time.monotonic() >= query_at:
                    with self.write_lock:
                        self.port.write(b'{"T":130}\n')
                    query_at = time.monotonic() + .2
                line = self.port.readline(4096)
                if not line:
                    continue
                data = json.loads(line)
                if not isinstance(data, dict) or data.get('T') != 1001:
                    continue
                if any(type(data.get(k)) not in (int, float) or not math.isfinite(data[k])
                       for k in ('L', 'R', 'v')):
                    continue
                with self.data_lock:
                    self.data, self.updated = data, time.monotonic()
            except (ValueError, UnicodeError):
                continue
            except Exception as error:
                if not self.quit.is_set():
                    self.error = str(error)
                    logging.exception('Telemetry failed')
                return

    def status(self):
        with self.data_lock:
            data = self.data
            if data and isinstance(data, dict):
                # Remap physical channels: firmware 'odl'/speed 'L' is physical Right,
                # firmware 'odr'/speed 'R' is physical Left.
                data = {
                    **data,
                    'odl': data.get('odr'),
                    'odr': data.get('odl'),
                    'L': data.get('R'),
                    'R': data.get('L'),
                }
            return {'data': data, 'age_s': time.monotonic() - self.updated,
                    'error': self.error, 'battery_voltage_v': self.data['v'] / 100 if self.data else None}

    def close(self):
        self.quit.set()
        try:
            self.wheels(0, 0)
        finally:
            self.port.cancel_read()
            self.thread.join(.5)
            self.port.close()
