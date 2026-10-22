#!/usr/bin/env python3
"""Add local-only finite bench controls to the reviewed telemetry app."""
from pathlib import Path
import sys
root = Path(sys.argv[1])
p = root / 'app.py'
text = p.read_text()
assert 'from telemetry_ctrl import BaseController' in text
text = text.replace('from telemetry_ctrl import BaseController', 'from bounded_ctrl import BaseController', 1)
text = text.replace('camera_stream = CameraStream(cvf.frame_process)', "camera_stream = CameraStream(cvf.frame_process, on_failure=lambda: base.guard.stop('camera failure'))\nbase.observation_ok = lambda: camera_stream.snapshot() is not None", 1)
text = text.replace("mode='telemetry-only'", "mode='supervised-bench'", 1)
text = text.replace('TELEMETRY ONLY: GPIO UART connected; all movement and peripheral commands disabled', 'GUARDED BENCH: motion disarmed by default; local finite command API only', 1)
insert = '''
# Available only over local SSH: LAN clients cannot issue movement here.
@app.route('/api/drive', methods=['POST'])
def guarded_drive():
    if request.remote_addr not in ('127.0.0.1', '::1'):
        return jsonify(error='Control requires local SSH access'), 403
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or set(data) != {'left', 'right', 'duration', 'owner'}:
        return jsonify(error='Expected left, right, duration, owner'), 400
    try:
        result = base.guard.drive(data['left'], data['right'], data['duration'], data['owner'])
        return jsonify(result)
    except ValueError as exc:
        return jsonify(error=str(exc)), 400
    except PermissionError as exc:
        return jsonify(error=str(exc)), 423
    except Exception:
        logging.exception('Guarded drive transport failure')
        return jsonify(error='Motor transport failed; Stop attempted'), 503

@app.route('/api/stop', methods=['POST'])
def guarded_stop():
    if request.remote_addr not in ('127.0.0.1', '::1'):
        return jsonify(error='Control requires local SSH access'), 403
    try:
        base.guard.stop('explicit API stop')
        return jsonify(base.guard.status())
    except Exception:
        logging.exception('Stop transport failure')
        return jsonify(error='Stop transport failed'), 503

'''
text = text.replace("# Route to render the HTML template", insert + "# Route to render the HTML template", 1)
p.write_text(text)
p = root / 'templates/index.html'
text = p.read_text().replace('Live camera and chassis telemetry. Motion disabled; waiting for supervised bench test.', 'Camera and telemetry. Guarded bench test via SSH only; web driving disabled.')
p.write_text(text)
print('Prepared guarded local bench API; web controls remain disabled')
