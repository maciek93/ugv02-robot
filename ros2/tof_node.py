#!/usr/bin/env python3
"""Publish measured VL53L5CX zones from the USB MCP2221A bridge."""
import json
import math
import os
import struct
import threading
import time

os.environ['BLINKA_MCP2221'] = '1'
os.environ['BLINKA_MCP2221_RESET_DELAY'] = '-1'

import board
import busio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vl53l5cx.cp import VL53L5CXCP
from vl53l5cx import DATA_DISTANCE_MM, DATA_TARGET_STATUS, Results, _DISTANCE_IDX, _TARGET_STATUS_IDX


class USBVL53L5CX(VL53L5CXCP):
    """Adapt the older Python ULD to MCP2221A and ST's current MCU boot rule."""

    def _wr_multi(self, register, data):
        for offset in range(0, len(data), 58):
            super()._wr_multi(register + offset, data[offset:offset + 58])

    def _rd_multi(self, register, size):
        result = bytearray()
        for offset in range(0, size, 60):
            result.extend(super()._rd_multi(register + offset, min(60, size - offset)))
        return result

    def _poll_for_answer(self, size, pos, register, mask, value):
        if register == 0x0006 and value == 0:
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                status = self._rd_byte(0x0006)
                if status & 0x80:
                    raise RuntimeError(f'MCU boot error: 0x{self._rd_byte(0x0007):02x}')
                if status & 1:
                    return 0
                time.sleep(.001)
            raise RuntimeError('MCU boot timed out')
        return super()._poll_for_answer(size, pos, register, mask, value)

    def get_ranging_data(self):
        buf = self._rd_multi(0, self._data_read_size)
        self._streamcount = buf[0]
        self._swap_buffer(buf)
        results = Results()
        offset = 16
        while offset + 4 <= len(buf):
            header = struct.unpack_from('<I', buf, offset)[0]
            kind, size, index = self._header(header)
            payload_size = kind * size if 1 < kind < 13 else size
            offset += 4
            if offset + payload_size > len(buf):
                break
            raw = buf[offset:offset + payload_size]
            if index == _DISTANCE_IDX:
                results.distance_mm = [max(0, x // 4) for x in struct.unpack(f'<{payload_size // 2}h', raw)]
            elif index == _TARGET_STATUS_IDX:
                results.target_status = list(raw)
            offset += payload_size
        return results


class ToFNode(Node):
    def __init__(self):
        super().__init__('ugv_vl53l5cx')
        self.publisher = self.create_publisher(String, '/tof/grid', 10)
        self.running = True
        self.worker = threading.Thread(target=self.read_loop, daemon=True)
        self.worker.start()

    def read_loop(self):
        while self.running and rclpy.ok():
            sensor = None
            try:
                bus = busio.I2C(board.SCL, board.SDA, frequency=400000)
                sensor = USBVL53L5CX(bus)
                if not sensor.is_alive():
                    raise RuntimeError('VL53L5CX chip ID not found')
                sensor.init()
                sensor.resolution = 64
                sensor.ranging_freq = 5
                sensor.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})
                sequence = 0
                last_frame = time.monotonic()
                while self.running and rclpy.ok():
                    if not sensor.check_data_ready():
                        if time.monotonic() - last_frame > 3:
                            raise RuntimeError('VL53L5CX stopped producing frames')
                        time.sleep(.025)
                        continue
                    data = sensor.get_ranging_data()
                    distances, status = data.distance_mm, data.target_status
                    if not distances or not status or len(distances) != 64 or len(status) != 64:
                        raise RuntimeError('Incomplete 8x8 range frame')
                    # ST status 5 is valid; status 6 is valid without wrap check.
                    usable = [d if s in (5, 6) and d > 0 else None for d, s in zip(distances, status)]
                    output = {'sensor': 'VL53L5CX', 'transport': 'MCP2221A-I2C',
                              'stamp': time.time(), 'sequence': sequence,
                              'resolution': [8, 8], 'distance_mm': distances,
                              'target_status': status, 'usable_distance_mm': usable,
                              'valid_zones': sum(v is not None for v in usable)}
                    msg = String()
                    msg.data = json.dumps(output, allow_nan=False)
                    self.publisher.publish(msg)
                    sequence += 1
                    last_frame = time.monotonic()
            except Exception as error:
                self.get_logger().error(f'VL53L5CX unavailable: {error}')
                time.sleep(2)
            finally:
                if sensor:
                    try: sensor.stop_ranging()
                    except Exception: pass

    def destroy_node(self):
        self.running = False
        self.worker.join(1)
        super().destroy_node()


def main():
    rclpy.init()
    node = ToFNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
