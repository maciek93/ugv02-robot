#!/usr/bin/env python3
"""Prove TX/RX by pausing telemetry flow and requesting one response; no motion."""
import json
import time
import serial

with serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1, write_timeout=0.5,
                   exclusive=True) as port:
    try:
        port.write(b'{"T":131,"cmd":0}\n')
        port.flush()
        time.sleep(0.4)
        port.reset_input_buffer()
        time.sleep(0.3)
        silent = port.read(max(1, min(port.in_waiting, 4096)))
        if silent:
            raise RuntimeError('Telemetry did not pause; TX not yet proven')
        print('Telemetry flow paused: quiet receive window verified')
        port.write(b'{"T":130}\n')
        port.flush()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = port.readline(4096)
            if not line:
                continue
            try:
                data = json.loads(line)
            except (ValueError, UnicodeError):
                continue
            if isinstance(data, dict) and data.get('T') == 1001:
                print('Requested telemetry:', json.dumps(data, sort_keys=True))
                print('PASS: bidirectional UART query/response; no movement sent')
                break
        else:
            raise RuntimeError('No telemetry response to T=130 query')
    finally:
        port.write(b'{"T":131,"cmd":1}\n')
        port.flush()
        print('Restored original telemetry flow')
