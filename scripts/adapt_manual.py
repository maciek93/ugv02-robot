#!/usr/bin/env python3
"""Add a dedicated guarded, local-only manual page; vendor raw routes stay blocked."""
from pathlib import Path
import sys
p=Path(sys.argv[1])/'app.py'
text=p.read_text()
marker='# Route to render the HTML template'
assert marker in text and 'manual_sessions = {}' not in text
text=text.replace("status.update(camera_ready=ready, mode='supervised-bench')", "status.update(camera_ready=ready, mode='supervised-bench', server_time_ms=time.time()*1000)")
insert='''
manual_sessions = {}
manual_lock = threading.RLock()

@app.route('/manual')
def manual_page():
    if request.remote_addr not in ('127.0.0.1', '::1'):
        return jsonify(error='Manual control requires an SSH tunnel from the Mac'), 403
    return render_template('manual.html')

@socketio.on('connect', namespace='/manual')
def manual_connect(auth=None):
    if request.remote_addr not in ('127.0.0.1', '::1'):
        return False
    with manual_lock:
        manual_sessions[request.sid] = 0

def manual_sequence(data, keys):
    if request.remote_addr not in ('127.0.0.1', '::1'):
        raise PermissionError('Manual control requires local SSH access')
    if not isinstance(data, dict) or set(data) != keys:
        raise ValueError('Invalid manual command fields')
    if data['connection'] != request.sid:
        raise ValueError('Command belongs to a previous connection')
    sequence = data['seq']
    if type(sequence) is not int or sequence <= manual_sessions.get(request.sid, float('inf')):
        raise ValueError('Stale or disconnected controller command')
    manual_sessions[request.sid] = sequence

@socketio.on('drive', namespace='/manual')
def manual_drive(data):
    with manual_lock:
        try:
            manual_sequence(data, {'left', 'right', 'duration', 'seq', 'connection', 'sent_at_ms'})
            duration, sent = data['duration'], data['sent_at_ms']
            if type(duration) not in (int, float) or not 0.05 <= duration <= 0.35:
                raise ValueError('Invalid command duration')
            if type(sent) not in (int, float):
                raise ValueError('Invalid command timestamp')
            age = time.time() - sent/1000
            if not 0 <= age <= duration - 0.05:
                raise ValueError('Expired movement request or invalid clock')
            return {'ok': True, **base.guard.drive(data['left'], data['right'], duration-age, request.sid)}
        except (ValueError, PermissionError) as exc:
            return {'ok': False, 'error': str(exc)}
        except Exception:
            logging.exception('Manual motor transport failed')
            return {'ok': False, 'error': 'Motor transport failed; Stop attempted'}

@socketio.on('stop', namespace='/manual')
def manual_stop(data):
    with manual_lock:
        try:
            manual_sequence(data, {'seq', 'connection'})
            base.guard.stop('manual Stop')
            return {'ok': True, **base.guard.status()}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

@socketio.on('disconnect', namespace='/manual')
def manual_disconnect(reason=None):
    with manual_lock:
        manual_sessions.pop(request.sid, None)
        base.guard.disconnect(request.sid)

'''
p.write_text(text.replace(marker,insert+marker,1))
print('Added guarded manual page; arming remains off by default')
