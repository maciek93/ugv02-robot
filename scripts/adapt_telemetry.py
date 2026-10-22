#!/usr/bin/env python3
"""Upgrade the reviewed stationary camera app to telemetry-only operation."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
def change(name, old, new):
    p = root / name
    text = p.read_text()
    if old not in text:
        raise RuntimeError(f'{name}: expected stage-one text missing')
    p.write_text(text.replace(old, new, 1))

change('app.py', 'from stationary_ctrl import BaseController', 'from telemetry_ctrl import BaseController')
change('app.py', "    return jsonify(camera_ready=ready, controller_connected=False,\n                   motion_enabled=False, mode='stationary'), 200 if ready else 503", "    status = base.status()\n    status.update(camera_ready=ready, mode='telemetry-only')\n    return jsonify(status), 200 if ready and status['controller_connected'] else 503")
change('app.py', "f['fb']['base_voltage']:None,", "f['fb']['base_voltage']:base.voltage_raw(),")
change('app.py', 'STATIONARY MODE: no serial port opened; all robot commands disabled', 'TELEMETRY ONLY: GPIO UART connected; all movement and peripheral commands disabled')
for _ in range(2):
    change('app.py', 'Robot commands disabled pending physical UART verification', 'Motion disabled; waiting for supervised bench test')
change('templates/index.html', 'Camera setup: driving disabled. UART, Stop and movement are not yet verified.', 'Live camera and chassis telemetry. Motion disabled; waiting for supervised bench test.')
# Remove unused peripheral startup branch entirely before any later control work.
p = root / 'app.py'
text = p.read_text()
start = text.index('    # No arm or pan/tilt is installed.')
end = text.index('    # feedback loop starts', start)
text = text[:start] + '    # No arm or pan/tilt installed; no peripheral startup commands.\n\n' + text[end:]
p.write_text(text)
print('Applied telemetry-only adaptations; no motion enabled')
