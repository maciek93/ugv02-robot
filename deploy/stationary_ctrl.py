"""Explicit no-hardware backend for camera bring-up before wiring verification.

No serial imports, port opens, writes or fabricated chassis telemetry.
Replace this backend only after physical checks and a read-only serial probe.
"""
import time

class BaseController:
    def __init__(self, *args):
        self.base_data = None
        self.extra_sensor = False
        self.use_lidar = False
        self.base_light_status = 0
        self.head_light_status = 0

    def feedback_data(self):
        time.sleep(0.2)
        return None

    def send_command(self, *args, **kwargs):
        return False

    base_json_ctrl = send_command
    base_speed_ctrl = send_command
    breath_light = send_command
    base_oled = send_command
    gimbal_ctrl = send_command
    lights_ctrl = send_command
    bus_servo_torque_lock = send_command
    bus_servo_id_set = send_command
    bus_servo_mid_set = send_command
    base_lights_ctrl = send_command
