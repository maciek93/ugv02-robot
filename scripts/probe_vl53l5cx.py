#!/usr/bin/env python3
"""Diagnose VL53L5CX through an MCP2221A without claiming ranging success.

This uses the temporary upstream Python ULD checkout installed on the Pi by the
sensor investigation. It deliberately prints the first failed register poll.
"""
import argparse
import json
import os
import struct
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=int, default=100000)
    parser.add_argument('--resolution', type=int, choices=(16, 64), default=64)
    parser.add_argument('--frames', type=int, default=3)
    args = parser.parse_args()
    os.environ['BLINKA_MCP2221'] = '1'
    os.environ['BLINKA_MCP2221_RESET_DELAY'] = '-1'

    import board
    import busio
    from vl53l5cx.cp import VL53L5CXCP
    from vl53l5cx import DATA_DISTANCE_MM, DATA_TARGET_STATUS
    from vl53l5cx import Results, _DISTANCE_IDX, _TARGET_STATUS_IDX

    bus = busio.I2C(board.SCL, board.SDA, frequency=args.frequency)

    class Sensor(VL53L5CXCP):
        def get_ranging_data(self):
            # Match ST ULD: swap each 32-bit word before interpreting blocks.
            # The older Python port parses raw words directly, pairing status
            # values with the wrong distance zones.
            buf = self._rd_multi(0, self._data_read_size)
            self._streamcount = buf[0]
            self._swap_buffer(buf)
            results = Results()
            offset = 16
            while offset + 4 <= len(buf):
                block = struct.unpack_from('<I', buf, offset)[0]
                block_type, size, index = self._header(block)
                payload_size = block_type * size if 1 < block_type < 13 else size
                offset += 4
                if offset + payload_size > len(buf):
                    break
                payload = buf[offset:offset + payload_size]
                if index == _DISTANCE_IDX:
                    results.distance_mm = [max(0, v // 4) for v in struct.unpack(
                        f'<{payload_size // 2}h', payload)]
                elif index == _TARGET_STATUS_IDX:
                    results.target_status = list(payload)
                offset += payload_size
            return results

        def _wr_multi(self, register, data):
            # MCP2221A has a 60-byte I2C transaction limit. Every chunk needs
            # its own 16-bit register address; slicing a giant write does not.
            for offset in range(0, len(data), 58):
                super()._wr_multi(register + offset, data[offset:offset + 58])

        def _rd_multi(self, register, size):
            result = bytearray()
            for offset in range(0, size, 60):
                result.extend(super()._rd_multi(register + offset, min(60, size - offset)))
            return result

        def _poll_for_answer(self, size, pos, register, mask, value):
            # This older Python port expects GO2 status 0 after MCU restart.
            # ST's current ULD considers bit 0 == 1 a successful MCU boot.
            if register == 0x0006 and value == 0:
                deadline = time.monotonic() + .5
                while time.monotonic() < deadline:
                    status = self._rd_byte(0x0006)
                    if status & 0x80:
                        raise RuntimeError(f'MCU boot error, status=0x{self._rd_byte(0x0007):02x}')
                    if status & 1:
                        print(json.dumps({'mcu_boot_status': status}), flush=True)
                        return 0
                    time.sleep(.001)
                raise RuntimeError('MCU boot timed out')
            try:
                return super()._poll_for_answer(size, pos, register, mask, value)
            except ValueError:
                actual = list(self._rd_multi(register, size))
                raise RuntimeError(f'poll register=0x{register:04x} expected index {pos} '
                                   f'& 0x{mask:02x} == 0x{value:02x}, got {actual}') from None

    sensor = Sensor(bus)
    print(json.dumps({'sensor': 'VL53L5CX', 'bus': 'MCP2221A', 'address': '0x29',
                      'chip_id_valid': sensor.is_alive(), 'frequency_hz': args.frequency}), flush=True)
    if not sensor.is_alive():
        return 2
    t0 = time.monotonic()
    sensor.init()
    print(json.dumps({'firmware_initialized': True, 'seconds': round(time.monotonic() - t0, 2)}), flush=True)
    sensor.resolution = args.resolution
    sensor.ranging_freq = 2
    sensor.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})
    deadline = time.monotonic() + max(10, args.frames * 3)
    frames = 0
    while time.monotonic() < deadline and frames < args.frames:
        if sensor.check_data_ready():
            data = sensor.get_ranging_data()
            print(json.dumps({'resolution': args.resolution, 'distance_mm': list(data.distance_mm),
                              'target_status': list(data.target_status)}), flush=True)
            frames += 1
            continue
        time.sleep(.05)
    sensor.stop_ranging()
    if frames == args.frames:
        return 0
    print(json.dumps({'error': f'Only {frames}/{args.frames} range frames received'}), flush=True)
    return 3


if __name__ == '__main__':
    sys.exit(main())
