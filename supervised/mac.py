"""Foreground Mac gateway, human approval UI, scoped agent API and session workers."""
import argparse
import base64
import functools
import fcntl
import hashlib
import hmac
import json
import logging
import multiprocessing
import os
from pathlib import Path
import secrets
import signal
import subprocess
import threading
import time

import requests
from flask import Flask, jsonify, make_response, request, send_file
from werkzeug.serving import make_server

from camera import atomic_json
from exploration import Explorer
from metric_tools import MetricTools

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / '.runtime'


class Gateway:
    def __init__(self, url, keys, ssh=True):
        self.url, self.keys, self.ssh = url, keys, ssh
        self.worker = None
        self.job = None
        self.human_seen = 0
        self.session = None
        self.mode = 'bench'
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.server = None
        self.csrf = secrets.token_urlsafe(32)
        self.browser = secrets.token_urlsafe(32)
        self.agent_key = secrets.token_urlsafe(32)
        (RUNTIME / 'agent-key').write_text(self.agent_key)
        (RUNTIME / 'agent-key').chmod(0o600)
        self.explorer = Explorer(self, ROOT, RUNTIME)
        self.metric = MetricTools(self, ROOT)
        self.heartbeat = threading.Thread(target=self.keepalive, daemon=True)
        self.heartbeat.start()

    def rpc(self, path, data=None, role='agent'):
        function = requests.get if data is None else requests.post
        args = {'headers': {'X-Robot-Key': self.keys[role]}, 'timeout': .08 if role == 'navigation' else 2}
        if data is not None:
            args['json'] = data
        response = function(self.url + path, **args)
        result = response.json()
        if not response.ok:
            raise PermissionError(result.get('error', 'Robot request failed'))
        return result

    def stop_motion(self, reason='cancelled'):
        # Cancel local renewals before attempting network I/O. A failed Stop
        # request must not allow the navigator to resume after reconnection.
        self.explorer.cancel.set()
        try:
            return self.rpc('/stop', {})
        finally:
            self.explorer.stop(reason)
            try:
                self.metric.ros('/stop', {})
            except Exception:
                pass  # Pi STOP already invalidated movement authority.

    def record(self, event, **data):
        with (ROOT / 'records/supervised-actions.jsonl').open('a') as output:
            output.write(json.dumps({'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                                     'event': event, **data}, allow_nan=False) + '\n')

    def keepalive(self):
        while not self.quit.wait(.15):
            if self.mode == 'manual':
                continue
            if self.session and time.monotonic() - self.human_seen < .45:
                try:
                    reply = self.rpc('/operator/heartbeat', {'session': self.session}, role='operator')
                    if self.explorer.mission and self.mode == 'explore' and reply.get('mission_id') != self.explorer.mission['id']:
                        self.explorer.stop(reply.get('last_reason', 'mission cancelled'))
                        self.session_operation('pause', self.mode)
                    if self.mode == 'mapping' and self.metric.manifest and self.metric.manifest['status'] == 'active' and reply.get('metric_mission_id') != self.metric.manifest['mission']['id']:
                        self.metric.interrupt(reply.get('last_reason', 'metric mission cancelled'))
                        self.session_operation('pause', self.mode)
                except Exception:
                    pass  # Pi's independent heartbeat timeout stops and pauses.
            elif self.session and time.monotonic() - self.human_seen > 1.5:
                try:
                    self.session_operation('pause', self.mode)
                except Exception:
                    self.session = None
                    self.stop_worker()

    def stop_worker(self):
        if self.worker:
            self.worker.terminate()
            self.worker.join(1)
            if self.worker.is_alive():
                self.worker.kill()
                self.worker.join(.5)
            self.worker = None
        self.explorer.stop('session stopped')
        for name in ('frame.jpg', 'depth.jpg', 'vision.json', 'navigation.json'):
            (RUNTIME / name).unlink(missing_ok=True)

    def session_operation(self, operation, mode):
        with self.lock:
            self.explorer.cancel.set()
            if operation in ('pause', 'end'):
                self.metric.interrupt('session ' + operation)
            result = self.rpc('/operator/session', {'operation': operation, 'mode': mode}, role='operator')
            self.session = None
            self.stop_worker()
            if operation in ('start', 'resume'):
                self.explorer.mission = None
                self.session, self.mode = result['session'], result['mode']
                self.human_seen = time.monotonic()
                if self.mode not in ('manual', 'mapping'):
                    from vision import run
                    self.worker = multiprocessing.get_context('spawn').Process(target=run, args=(
                        self.url, self.keys['vision'], self.session, str(RUNTIME),
                        str(ROOT / 'models/depth-v2/DepthAnythingV2SmallF16.mlpackage'),
                        str(ROOT / 'models/yolo26n.pt')))
                    self.worker.start()
            self.record('session', operation=operation, mode=mode)
            return result

    def execute(self, proposal_id):
        with self.lock:
            if self.job and self.job.is_alive():
                raise PermissionError('A leg is already running')
            result = self.rpc('/execute', {'proposal_id': proposal_id})
            self.record('execute', proposal=result)
            self.job = threading.Thread(target=self.drive_leg, args=(proposal_id,), daemon=True)
            self.job.start()
            return result

    def drive_leg(self, proposal_id):
        started = time.monotonic()
        try:
            while not self.quit.is_set() and time.monotonic() - started < 120:
                health = self.rpc('/health')
                job = health['execution']
                if not job or job['id'] != proposal_id:
                    return
                if health['blocking_reason']:
                    raise PermissionError(health['blocking_reason'])
                if job['action'].startswith('continuous_') or job['action'].startswith('pwm_'):
                    try:
                        result = self.rpc('/renew', {'proposal_id': proposal_id})
                    except PermissionError as error:
                        if str(error) == 'No matching approved continuous trial':
                            ended = self.rpc('/health')
                            if not ended['moving'] and ended['last_reason'] == 'continuous trial completed':
                                return
                        if str(error) != 'Observe after Stop before moving again':
                            raise
                    else:
                        self.record('renew', proposal_id=proposal_id, **result)
                        if result.get('done'):
                            return
                    time.sleep(.04)
                    continue
                vision = health['vision']
                if not health['moving'] and vision and vision['frame_id'] != job['last_frame']:
                    try:
                        result = self.rpc('/pulse', {'proposal_id': proposal_id})
                    except PermissionError as error:
                        # Only settling/freshness rejections can be retried; hazards abort the leg.
                        if str(error) not in ('Wait for Stop and settling before another pulse',
                                               'Observe after Stop before moving again',
                                               'A fresh observation is required between pulses'):
                            raise
                    else:
                        self.record('pulse', proposal_id=proposal_id, **result)
                        if result.get('done'):
                            return
                time.sleep(.04)
            raise PermissionError('Leg exceeded time budget')
        except Exception as error:
            self.record('leg_aborted', reason=str(error))
        finally:
            try:
                result = self.rpc('/stop', {})
                self.record('leg_stopped', reason=result['last_reason'], chassis=result['chassis'])
            except Exception as error:
                self.record('stop_unreachable', reason=str(error))

    def close(self):
        self.quit.set()
        try:
            self.session_operation('end', self.mode)
        except Exception:
            logging.exception('Remote End unavailable; Pi watchdog remains active')
        self.stop_worker()
        self.heartbeat.join(.5)
        (RUNTIME / 'agent-key').unlink(missing_ok=True)


def create_app(gateway):
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = 65536

    def agent(function):
        @functools.wraps(function)
        def wrapped(*args, **kwargs):
            if not hmac.compare_digest(request.headers.get('X-Agent-Key', ''), gateway.agent_key):
                return jsonify(error='Agent credential required'), 403
            return function(*args, **kwargs)
        return wrapped

    def human(function):
        @functools.wraps(function)
        def wrapped(*args, **kwargs):
            if (not hmac.compare_digest(request.cookies.get('human', ''), gateway.browser)
                    or not hmac.compare_digest(request.headers.get('X-CSRF', ''), gateway.csrf)
                    or request.headers.get('Origin') != 'http://127.0.0.1:5001'):
                return jsonify(error='Use the human control page'), 403
            return function(*args, **kwargs)
        return wrapped

    @app.before_request
    def localhost():
        if request.host != '127.0.0.1:5001' or request.remote_addr != '127.0.0.1':
            return jsonify(error='Local Mac access only'), 403

    @app.errorhandler(PermissionError)
    @app.errorhandler(ValueError)
    def rejected(error):
        return jsonify(error=str(error)), 423

    @app.errorhandler(requests.RequestException)
    def offline(error):
        return jsonify(error='Robot connection unavailable'), 503

    @app.get('/')
    def page():
        # Human page exposes no agent approval operation. Local browser is the human boundary.
        html = (ROOT / 'supervised/control.html').read_text().replace('__CSRF__', gateway.csrf)
        response = make_response(html)
        response.set_cookie('human', gateway.browser, httponly=True, samesite='Strict')
        response.headers['Cache-Control'] = 'no-store'
        response.headers['Content-Security-Policy'] = "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; frame-ancestors 'none'"
        return response

    @app.get('/status')
    def status():
        with gateway.lock:
            result = gateway.rpc('/health')
            result['gateway_protocol'] = 3
            result['exploration'] = gateway.explorer.status()
            result['metric_survey'] = gateway.metric.manifest
            if result['state'] != 'active' and gateway.worker:
                gateway.session = None
                gateway.stop_worker()
            try:
                result['local_vision'] = json.loads((RUNTIME / 'vision.json').read_text())
            except (OSError, ValueError):
                result['local_vision'] = None
            return jsonify(result)

    @app.get('/image/<kind>')
    def image(kind):
        if kind not in ('frame', 'depth'):
            return '', 404
        if kind == 'frame':
            try:
                observation = gateway.rpc('/observe', role='vision')
                response = make_response(base64.b64decode(observation['jpeg']))
                response.headers['Content-Type'] = 'image/jpeg'
                response.headers['Cache-Control'] = 'no-store'
                return response
            except Exception:
                return '', 404
        if not gateway.worker:
            return '', 404
        path = RUNTIME / (kind + '.jpg')
        if not path.exists():
            return '', 404
        response = send_file(path, mimetype='image/jpeg', max_age=0)
        response.headers['Cache-Control'] = 'no-store'
        return response

    @app.post('/human/heartbeat')
    @human
    def heartbeat():
        gateway.human_seen = time.monotonic()
        return jsonify(ok=True)

    @app.post('/agent/session')
    @agent
    def session():
        data = request.get_json()
        result = gateway.session_operation(data['operation'], data.get('mode', 'bench'))
        if data['operation'] == 'end':
            threading.Timer(.3, gateway.server.shutdown).start()
        return jsonify(result)

    @app.post('/human/stop')
    @human
    def human_stop():
        requested = time.monotonic()
        result = gateway.stop_motion('cancelled by web STOP')
        gateway.record('web_stop', request_monotonic=requested,
                       reply_monotonic=time.monotonic(), moving_after=result['moving'])
        return jsonify(result)

    @app.post('/agent/room')
    @agent
    def room():
        data = request.get_json()
        rooms = ('library', 'kitchen', 'living room', 'corridor')
        if data['room'] not in rooms or data.get('from_room') not in (*rooms, None, ''):
            raise ValueError('Unknown room')
        path = ROOT / 'records/room-map.json'
        graph = json.loads(path.read_text()) if path.exists() else {'rooms': {}, 'connections': []}
        observation = gateway.rpc('/observe')
        destination = ROOT / 'records' / ('room-' + data['room'].replace(' ', '-') + '.jpg')
        destination.write_bytes(base64.b64decode(observation.pop('jpeg')))
        graph['rooms'][data['room']] = {'reference': destination.name, 'observation': observation}
        if data.get('from_room'):
            edge = [data['from_room'], data['room']]
            if edge not in graph['connections']:
                graph['connections'].append(edge)
        atomic_json(path, graph)
        return jsonify(graph)

    @app.get('/agent/observe')
    @agent
    def observe():
        result = gateway.rpc('/observe')
        result['status'] = gateway.rpc('/health')
        path = ROOT / 'records/room-map.json'
        result['room_map'] = json.loads(path.read_text()) if path.exists() else None
        return jsonify(result)

    @app.get('/agent/status')
    @agent
    def agent_status():
        return status()

    @app.post('/agent/propose')
    @agent
    def propose():
        result = gateway.rpc('/propose', request.get_json())
        gateway.record('proposed', proposal=result)
        return jsonify(result)

    @app.post('/agent/execute')
    @agent
    def execute():
        return jsonify(gateway.execute(request.get_json()['proposal_id']))

    @app.post('/agent/stop')
    @agent
    def stop():
        result = gateway.stop_motion('cancelled')
        return jsonify(result)

    @app.post('/agent/mission/propose')
    @agent
    def mission_propose():
        result = gateway.rpc('/mission/propose', request.get_json())
        gateway.record('mission_proposed', proposal=result)
        return jsonify(result)

    @app.post('/agent/mission/start')
    @agent
    def mission_start():
        return jsonify(gateway.explorer.start(request.get_json()['mission_id']))

    @app.get('/agent/mission/candidates')
    @agent
    def mission_candidates():
        return jsonify(gateway.explorer.candidates())

    @app.post('/agent/mission/select')
    @agent
    def mission_select():
        return jsonify(gateway.explorer.choose(request.get_json()))

    @app.post('/agent/mission/annotate')
    @agent
    def mission_annotate():
        return jsonify(gateway.explorer.annotate(request.get_json()))

    @app.get('/agent/mission/status')
    @agent
    def mission_status():
        h = gateway.rpc('/health')
        if gateway.explorer.mission and not h['mission']['active']:
            gateway.explorer.stop(h['last_reason'])
        return jsonify(pi=h['mission'], navigator=gateway.explorer.status())

    @app.post('/agent/mission/finish')
    @agent
    def mission_finish():
        outcome = 'cancelled' if gateway.explorer.status()['running'] else 'completed'
        gateway.stop_motion(outcome)
        gateway.session_operation('pause', gateway.mode)
        return jsonify(gateway.explorer.status())

    @app.post('/agent/metric/mission/propose')
    @agent
    def metric_mission_propose():
        result = gateway.rpc('/metric/mission/propose', request.get_json())
        gateway.record('metric_mission_proposed', proposal=result)
        return jsonify(result)

    @app.post('/agent/metric/mission/start')
    @agent
    def metric_mission_start():
        return jsonify(gateway.metric.start(request.get_json()['mission_id']))

    @app.get('/agent/metric/status')
    @agent
    def metric_status():
        return jsonify(gateway.metric.status())

    @app.post('/agent/metric/view_metric_map')
    @agent
    def view_metric_map():
        return jsonify(gateway.metric.view_map(request.get_json() or {}))

    @app.get('/agent/metric/view_lidar_scan')
    @agent
    def view_lidar_scan():
        return jsonify(gateway.metric.view_scan())

    @app.get('/agent/metric/view_depth_grid')
    @agent
    def view_depth_grid():
        return jsonify(gateway.metric.view_depth())

    @app.post('/agent/metric/inspect_map_region')
    @agent
    def inspect_map_region():
        return jsonify(gateway.metric.inspect_region(request.get_json()))

    @app.post('/agent/metric/preview_route')
    @agent
    def preview_route():
        return jsonify(gateway.metric.preview_route(request.get_json()))

    @app.post('/agent/metric/follow_route')
    @agent
    def follow_route():
        return jsonify(gateway.metric.command('follow_route', request.get_json()))

    @app.post('/agent/metric/navigate_to')
    @agent
    def navigate_to():
        return jsonify(gateway.metric.command('navigate_to', request.get_json()))

    @app.post('/agent/metric/drive_distance')
    @agent
    def drive_distance():
        return jsonify(gateway.metric.command('drive_distance', request.get_json()))

    @app.post('/agent/metric/turn_angle')
    @agent
    def turn_angle():
        return jsonify(gateway.metric.command('turn_angle', request.get_json()))

    @app.post('/agent/metric/annotate')
    @agent
    def metric_annotate():
        return jsonify(gateway.metric.annotate(request.get_json()))

    @app.post('/agent/metric/capture_view')
    @agent
    def metric_capture_view():
        gateway.metric.approved()
        return jsonify(gateway.metric.checkpoint('requested', gateway.metric.ros('/state').get('pose')))

    @app.post('/agent/metric/pause')
    @agent
    def metric_pause():
        gateway.stop_motion('metric mission paused')
        gateway.metric.interrupt('paused')
        return jsonify(gateway.session_operation('pause', 'mapping'))

    @app.post('/agent/metric/stop')
    @agent
    def metric_stop():
        result = gateway.stop_motion('metric mission stopped')
        gateway.metric.interrupt('stopped')
        return jsonify(result)

    @app.post('/agent/metric/finish')
    @agent
    def metric_finish():
        result = gateway.metric.finish()
        gateway.stop_motion('metric mission completed')
        gateway.session_operation('pause', 'mapping')
        return jsonify(result)

    @app.get('/agent/metric/report')
    @agent
    def metric_report():
        directory = gateway.metric.directory
        if not directory or not (directory / 'report.md').exists():
            raise PermissionError('No completed metric survey report')
        path = directory / 'report.md'
        return jsonify(report=path.read_text(), path=str(path.resolve()))

    @app.get('/agent/mission/report')
    @agent
    def mission_report():
        j = gateway.explorer.journal
        if not j: raise ValueError('No mission journal in this session')
        path = j.directory / 'report.md'
        return jsonify(manifest=j.manifest, report_path=str(path.resolve()) if path.exists() else None,
                       report=path.read_text() if path.exists() else None)

    @app.post('/agent/end')
    @agent
    def end():
        result = gateway.session_operation('end', gateway.mode)
        threading.Timer(.3, gateway.server.shutdown).start()
        return jsonify(result)

    return app


def main():
    logging.getLogger('werkzeug').setLevel(logging.WARNING)
    RUNTIME.mkdir(mode=0o700, exist_ok=True)
    os.umask(0o077)
    singleton = (RUNTIME / 'gateway.lock').open('a')
    fcntl.flock(singleton, fcntl.LOCK_EX | fcntl.LOCK_NB)
    (RUNTIME / 'gateway.pid').write_text(str(os.getpid()))
    keys = json.loads((RUNTIME / 'pi-keys.json').read_text())
    forwarding = ['ssh', '-S', str(RUNTIME / 'ssh-control'), '-o', 'BatchMode=yes']
    subprocess.run(forwarding + ['-O', 'forward', '-L', '127.0.0.1:5002:127.0.0.1:5000',
                                'ugvpi@ugvpi.local'], check=True)
    subprocess.run(forwarding + ['-O', 'forward', '-L', '127.0.0.1:5003:127.0.0.1:5003',
                                'ugvpi@ugvpi.local'], check=True)
    gateway = Gateway('http://127.0.0.1:5002', keys)
    server = make_server('127.0.0.1', 5001, create_app(gateway), threaded=True)
    server.daemon_threads = True
    gateway.server = server

    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print('Camera + STOP: http://127.0.0.1:5001/ — sessions controlled from chat', flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    finally:
        gateway.close()
        server.server_close()
        try:
            for command in (
                ['ssh', '-S', str(RUNTIME / 'ssh-control'), '-o', 'BatchMode=yes',
                 'ugvpi@ugvpi.local', 'sudo systemctl stop ugv-supervised'],
                ['ssh', '-S', str(RUNTIME / 'ssh-control'), '-O', 'cancel',
                 '-L', '127.0.0.1:5002:127.0.0.1:5000', 'ugvpi@ugvpi.local'],
                ['ssh', '-S', str(RUNTIME / 'ssh-control'), '-O', 'cancel',
                 '-L', '127.0.0.1:5003:127.0.0.1:5003', 'ugvpi@ugvpi.local']):
                try:
                    subprocess.run(command, timeout=3)
                except subprocess.TimeoutExpired:
                    logging.warning('SSH shutdown timed out; completing local cleanup')
        finally:
            (RUNTIME / 'gateway.pid').unlink(missing_ok=True)
            singleton.close()


if __name__ == '__main__':
    main()
