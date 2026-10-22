#!/usr/bin/env python3
"""Telemetry-only ESP32 probe. Sends only the reviewed T=130 query."""
import argparse
import json
import time
import serial

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='/dev/ttyAMA0')
parser.add_argument('--baud', type=int, default=115200)
args = parser.parse_args()
print(f'Telemetry-only probe: {args.port}, {args.baud} baud; command T=130')
found = False
with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=0.5,
                   exclusive=True, rtscts=False, dsrdtr=False) as port:
    port.reset_input_buffer()
    for attempt in range(3):
        port.write(b'{"T":130}\n')
        port.flush()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            line = port.readline(4096)
            if not line:
                continue
            try:
                data = json.loads(line)
            except (ValueError, UnicodeError):
                print('Non-JSON RX:', repr(line[:160]))
                continue
            print('RX:', json.dumps(data, sort_keys=True))
            if isinstance(data, dict) and data.get('T') == 1001:
                found = True
        if found:
            break
print('PASS: chassis telemetry received' if found else 'FAIL: no T=1001 telemetry response')
raise SystemExit(0 if found else 2)
