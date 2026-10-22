"""Codex-facing measured-map tools and persistent metric mission journal."""
import base64
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import secrets
import threading
import time

from PIL import Image, ImageDraw
import requests

from camera import atomic_json
from metric_map import MetricGrid


def utc():
    return datetime.now(timezone.utc).isoformat()


def validate_route_scope(mission, current_room, waypoints):
    room = current_room
    transitions = []
    for waypoint in waypoints:
        next_room = waypoint.get('room')
        if next_room not in mission['rooms']:
            raise PermissionError('Every waypoint needs a room in the approved mission')
        if next_room != room:
            doorway = {'id': waypoint.get('via_doorway'), 'from_room': room,
                       'to_room': next_room}
            if doorway not in mission['doorways']:
                raise PermissionError('Route crosses a doorway outside the approved direction or scope')
            transitions.append(doorway)
            room = next_room
    return room, transitions


class MetricTools:
    def __init__(self, gateway, root):
        self.gateway = gateway
        self.root = Path(root)
        self.directory = None
        self.manifest = None
        self.previews = {}
        self.pending_goals = {}
        self.seen_result = None
        self.lock = threading.RLock()
        for path in (self.root / 'data/metric_missions').glob('*/manifest.json'):
            try:
                saved = json.loads(path.read_text())
                if saved.get('status') == 'active':
                    saved.update(status='interrupted', finished=utc(),
                                 reason='Gateway restarted; movement authority not restored')
                    atomic_json(path, saved)
            except (ValueError, OSError):
                continue

    def ros(self, path, data=None, timeout=3):
        method = requests.get if data is None else requests.post
        url = 'http://127.0.0.1:5003' + path
        args = {'headers': {'X-Robot-Key': self.gateway.keys['navigation']}, 'timeout': timeout}
        if data is not None:
            args['json'] = data
        response = method(url, **args)
        value = response.json()
        if not response.ok:
            raise PermissionError(value.get('error', 'ROS bridge request failed'))
        return value

    def pi(self):
        return self.gateway.rpc('/metric/telemetry', role='navigation')

    def approved(self):
        pi = self.pi()
        mission = pi['mission']['active']
        if not mission or not self.manifest or mission['id'] != self.manifest['mission']['id']:
            raise PermissionError('Start the approved metric mission in this session')
        return mission, pi

    def start(self, mission_id):
        with self.lock:
            pi = self.pi()
            mission = pi['mission']['active']
            if pi['mode'] != 'mapping' or not mission or mission['id'] != mission_id:
                raise PermissionError('Metric mission must be approved through chat first')
            if self.manifest and self.manifest['status'] == 'active':
                raise PermissionError('Another metric mission is active')
            state = self.ros('/state')
            if not state.get('pose') or not state.get('map_version'):
                raise PermissionError('A current SLAM map and localisation are required')
            self.directory = self.root / 'data/metric_missions' / mission_id
            self.directory.mkdir(parents=True, exist_ok=False)
            self.manifest = {'version': 1, 'status': 'active', 'started': utc(),
                             'mission': mission, 'configuration_id': pi['profile']['configuration_id'],
                             'current_room': mission['start_room'],
                             'checkpoints': [], 'goals': [], 'places': [], 'obstacles': [],
                             'doorways_observed': [], 'transitions_traversed': [], 'uncertainties': []}
            self.save()
            self.snapshot('start')
            self.checkpoint('start', state['pose'])
            return self.status()

    def save(self):
        atomic_json(self.directory / 'manifest.json', self.manifest)

    def event(self, event, **data):
        if self.directory:
            with (self.directory / 'events.jsonl').open('a') as f:
                f.write(json.dumps({'time': utc(), 'event': event, **data}, allow_nan=False) + '\n')

    def snapshot(self, label):
        snapshot = self.ros('/map')
        MetricGrid(snapshot)
        path = self.directory / f'{label}-occupancy.json'
        atomic_json(path, snapshot)
        self.event('map_snapshot', path=path.name, map_version=snapshot['map_version'])
        return path

    def checkpoint(self, label, pose):
        if not self.directory:
            return None
        result = self.gateway.rpc('/observe')
        path = self.directory / f'{label}-{len(self.manifest["checkpoints"]):03}.jpg'
        path.write_bytes(base64.b64decode(result['jpeg']))
        record = {'time': utc(), 'label': label, 'pose': pose,
                  'reference': path.name, 'camera_frame_id': result.get('frame_id')}
        self.manifest['checkpoints'].append(record)
        self.save()
        self.event('checkpoint', **record)
        return record

    def status(self):
        with self.lock:
            bridge = self.ros('/state')
            result = bridge.get('result')
            if self.manifest and result and result.get('goal_id') != self.seen_result:
                self.seen_result = result['goal_id']
                result = {**result, **self.pending_goals.pop(result['goal_id'], {})}
                self.manifest['goals'].append(result)
                self.event('goal_finished', **result)
                if result['outcome'] == 'reached':
                    label = f'goal-{len(self.manifest["goals"]):03}'
                    for operation in (lambda: self.snapshot(label),
                                      lambda: self.checkpoint(label, bridge.get('pose'))):
                        try:
                            operation()
                        except Exception as error:
                            self.manifest['uncertainties'].append(f'{label} evidence capture failed: {error}')
                else:
                    self.manifest['uncertainties'].append(f"Goal {result['goal_id']}: {result['outcome']}")
                self.save()
            return {'pi': self.pi()['mission'], 'bridge': bridge,
                    'manifest': self.manifest, 'directory': str(self.directory) if self.directory else None}

    def grid(self):
        return MetricGrid(self.ros('/map'))

    def render_region(self, region, robot=None, route=None, grid=None):
        cells = region['cells']
        h, w = len(cells), len(cells[0])
        image = Image.new('RGB', (w, h))
        pixels = image.load()
        for row in range(h):
            for col in range(w):
                value = cells[row][col]
                pixels[col, h - 1 - row] = (48, 48, 55) if value < 0 else (230, 230, 230) if value < 50 else (35, 35, 35)
        image = image.resize((w * 4, h * 4), Image.Resampling.NEAREST)
        draw = ImageDraw.Draw(image)
        left, bottom = region['bottom_left_cell']
        resolution = region['resolution_m']
        grid = grid or self.grid()

        def position(x, y):
            col, row = grid.world_to_cell(x, y)
            return ((col - left + .5) * 4, (h - (row - bottom) - .5) * 4)

        if route:
            points = [position(p['x'], p['y']) for p in route]
            if len(points) > 1:
                draw.line(points, fill=(15, 130, 240), width=3)
            for x, y in points:
                draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=(15, 130, 240))
        if robot:
            x, y = position(robot['x'], robot['y'])
            draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=(235, 50, 50))
        return image

    def view_map(self, body):
        grid = self.grid()
        state = self.ros('/state')
        pose = state.get('pose')
        if not pose:
            raise PermissionError('Robot localisation unavailable')
        center_x = body.get('center_x', pose['x'])
        center_y = body.get('center_y', pose['y'])
        region = grid.region(center_x, center_y, body.get('width_m', 4.), body.get('height_m', 4.),
                             max_cells=200000)
        folder = self.directory or self.root / 'data/metric_maps'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f'map-{grid.version}-{secrets.token_hex(4)}.png'
        self.render_region(region, pose, grid=grid).save(path)
        return {'image': str(path.resolve()), 'region': region, 'robot_pose': pose,
                'map_version': grid.version, 'map_stamp': grid.stamp}

    def view_scan(self):
        scan = self.ros('/scan')
        state = self.ros('/state')
        img = Image.new('RGB', (600, 600), (30, 30, 35))
        draw = ImageDraw.Draw(img)
        center = 300
        for radius in (75, 150, 225):
            draw.ellipse((center-radius, center-radius, center+radius, center+radius), outline=(75, 75, 80))
        draw.line((center, 0, center, 600), fill=(60, 60, 65))
        draw.line((0, center, 600, center), fill=(60, 60, 65))
        max_range = min(4., scan['range_max_m'])
        for i, value in enumerate(scan['ranges_m']):
            if value is None or not 0 < value <= max_range:
                continue
            a = scan['angle_min'] + i * scan['angle_increment']
            x = center + 250 * value / max_range * math.cos(a)
            y = center - 250 * value / max_range * math.sin(a)
            draw.ellipse((x-2, y-2, x+2, y+2), fill=(40, 210, 230))
        draw.ellipse((295, 295, 305, 305), fill=(240, 70, 70))
        folder = self.directory or self.root / 'data/metric_maps'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f'scan-{scan["sequence"]}.png'
        img.save(path)
        return {'image': str(path.resolve()), 'map_version': state.get('map_version'),
                'robot_pose': state.get('pose'), 'range_unit': 'metre', **scan}

    def view_depth(self):
        depth = self.ros('/depth')
        state = self.ros('/state')
        return {'frame_id': 'tof_optical', 'range_unit': 'millimetre',
                'map_version': state.get('map_version'), 'robot_pose': state.get('pose'), **depth}

    def inspect_region(self, body):
        grid = self.grid()
        center_x, center_y = body['center_x'], body['center_y']
        region = grid.region(center_x, center_y, body.get('width_m', 2.), body.get('height_m', 2.))
        cells = [v for row in region['cells'] for v in row]
        left, bottom = region['bottom_left_cell']
        nearest = min((math.hypot(wx - center_x, wy - center_y)
                       for row, values in enumerate(region['cells'])
                       for col, value in enumerate(values) if value >= 50
                       for wx, wy in [grid.cell_to_world(left + col, bottom + row)]), default=None)
        pi = self.pi()
        p = pi.get('profile')
        footprint = grid.footprint(center_x, center_y, body.get('yaw', 0), p['length_m'], p['width_m'],
                                   p['margin_m'] + p['fallback_stop_m']) if p else None
        return {'region': region, 'free_cells': sum(0 <= v < 50 for v in cells),
                'occupied_cells': sum(v >= 50 for v in cells),
                'unknown_cells': sum(v < 0 for v in cells),
                'nearest_occupied_cell_m': nearest, 'footprint_clearance': footprint,
                'robot_pose': self.ros('/state').get('pose')}

    def preview_route(self, body):
        mission, pi = self.approved()
        room, transitions = validate_route_scope(mission, self.manifest['current_room'], body['waypoints'])
        grid = self.grid()
        pose = self.ros('/state')['pose']
        if not pose:
            raise PermissionError('Localisation unavailable')
        p = pi['profile']
        footprint = {'length_m': p['length_m'], 'width_m': p['width_m'],
                     'margin_m': p['margin_m'] + p['fallback_stop_m']}
        preview = grid.preview(pose, body['waypoints'], footprint)
        preview_id = secrets.token_hex(12)
        self.previews[preview_id] = {'created': time.monotonic(), 'map_version': grid.version,
                                     'pose': pose, 'waypoints': body['waypoints'],
                                     'mission_id': mission['id'], 'verdict': preview['verdict'],
                                     'transitions': transitions, 'destination_room': room}
        center_x = (pose['x'] + body['waypoints'][-1]['x']) / 2
        center_y = (pose['y'] + body['waypoints'][-1]['y']) / 2
        xs = [pose['x'], *[x['x'] for x in body['waypoints']]]
        ys = [pose['y'], *[x['y'] for x in body['waypoints']]]
        region = grid.region(center_x, center_y, min(20, max(xs)-min(xs)+2), min(20, max(ys)-min(ys)+2),
                             max_cells=200000)
        path = self.directory / f'route-preview-{preview_id}.png'
        self.render_region(region, pose, [pose, *body['waypoints']], grid).save(path)
        self.event('route_preview', preview_id=preview_id, **preview)
        return {'preview_id': preview_id, 'image': str(path.resolve()),
                'directed_transitions': transitions, 'destination_room': room, **preview}

    def command(self, operation, body):
        mission, _ = self.approved()
        state = self.ros('/state')
        if state.get('goal'):
            raise PermissionError('A goal is already running')
        map_version = state.get('map_version')
        if not map_version:
            raise PermissionError('Map unavailable')
        payload = {'mission_id': mission['id'], 'map_version': map_version,
                   'goal_id': secrets.token_hex(12)}
        if operation == 'follow_route':
            preview = self.previews.pop(body['preview_id'], None)
            if not preview or preview['mission_id'] != mission['id'] or preview['map_version'] != map_version or preview['verdict'] != 'clear' or time.monotonic()-preview['created'] > 30:
                raise PermissionError('Fresh clear route preview required')
            payload['waypoints'] = preview['waypoints']
        elif operation == 'navigate_to':
            if body['target'].get('room') != self.manifest['current_room']:
                raise PermissionError('Use a previewed route for a different room')
            payload['target'] = body['target']
        else:
            payload['value'] = body['value']
        result = self.ros('/' + operation, payload)
        self.pending_goals[result['goal_id']] = {'operation': operation,
                                                 'directed_transitions': preview['transitions'] if operation == 'follow_route' else [],
                                                 'destination_room': preview['destination_room'] if operation == 'follow_route' else self.manifest['current_room']}
        self.event('goal_started', operation=operation, **result)
        return result

    def annotate(self, body):
        mission, _ = self.approved()
        category = body.get('category')
        if category not in ('place', 'obstacle', 'observed_doorway', 'traversed_transition', 'uncertainty'):
            raise ValueError('Invalid annotation category')
        if not isinstance(body.get('label'), str) or not 1 <= len(body['label']) <= 200:
            raise ValueError('Annotation label required')
        if category == 'traversed_transition':
            doorway = {'id': body.get('doorway_id'), 'from_room': body.get('from_room'),
                       'to_room': body.get('to_room')}
            if doorway not in mission['doorways']:
                raise PermissionError('Transition is outside the approved directed doorway scope')
            reached = next((g for g in self.manifest['goals'] if g['outcome'] == 'reached'
                            and g['goal_id'] == body.get('reached_goal_id')), None)
            if not reached or doorway not in reached.get('directed_transitions', []):
                raise PermissionError('A reached route through this doorway is required as traversal evidence')
            if doorway['from_room'] != self.manifest['current_room']:
                raise PermissionError('Transition does not begin in the current recorded room')
        elif category == 'place' and body.get('room') not in mission['rooms']:
            raise PermissionError('Place must be labelled with an approved room')
        with self.lock:
            key = {'place': 'places', 'obstacle': 'obstacles',
                   'observed_doorway': 'doorways_observed',
                   'traversed_transition': 'transitions_traversed',
                   'uncertainty': 'uncertainties'}[category]
            entry = {'time': utc(), 'label': body['label'],
                     'pose': self.ros('/state').get('pose'), 'notes': body.get('notes', '')}
            if category == 'traversed_transition':
                entry.update(doorway_id=body['doorway_id'], from_room=body['from_room'],
                             to_room=body['to_room'], reached_goal_id=body['reached_goal_id'])
            elif category == 'place':
                entry['room'] = body['room']
            self.manifest[key].append(entry)
            if category == 'traversed_transition':
                self.manifest['current_room'] = body['to_room']
            self.save()
            self.event('annotation', category=category, **entry)
            return entry

    def finish(self):
        if not self.manifest or self.manifest['status'] != 'active':
            raise PermissionError('No active metric mission')
        self.status()
        confirmed = {(x['reached_goal_id'], x['doorway_id'])
                     for x in self.manifest['transitions_traversed']}
        for goal in self.manifest['goals']:
            if goal['outcome'] == 'reached':
                for doorway in goal.get('directed_transitions', []):
                    if (goal['goal_id'], doorway['id']) not in confirmed:
                        self.manifest['uncertainties'].append(
                            f"Doorway {doorway['id']} on reached goal {goal['goal_id']} was not confirmed as traversed")
        try:
            self.snapshot('finish')
        except Exception as error:
            self.manifest['uncertainties'].append(f'Final map snapshot failed: {error}')
        try: self.manifest['posegraph_export'] = self.ros('/save_map', {'mission_id': self.manifest['mission']['id']}, timeout=25)
        except Exception as error:
            self.manifest['uncertainties'].append(f'Pose-graph export failed: {error}')
        self.manifest.update(status='completed', finished=utc(), ending_pose=self.ros('/state').get('pose'))
        self.save()
        lines = ['# LiDAR survey', '', f"Outcome: **{self.manifest['status']}**",
                 f"Ending pose: `{self.manifest['ending_pose']}`", '', '## Checkpoints', '']
        if self.manifest.get('posegraph_export'):
            lines.insert(4, f"Pi map/pose graph: `{self.manifest['posegraph_export']}`")
        for c in self.manifest['checkpoints']:
            lines.append(f"- {c['label']}: [camera image]({c['reference']}); map pose {c['pose']}.")
        for title, key in (('Visited places', 'places'), ('Observed doorways', 'doorways_observed'),
                           ('Traversed transitions', 'transitions_traversed'), ('Obstacles', 'obstacles'),
                           ('Uncertainty and incomplete areas', 'uncertainties')):
            lines += ['', '## ' + title, '']
            lines.extend('- ' + json.dumps(x) for x in self.manifest[key])
        path = self.directory / 'report.md'
        path.write_text('\n'.join(lines) + '\n')
        self.event('mission_finished', report=path.name)
        return {'report': str(path.resolve()), 'manifest': self.manifest}

    def interrupt(self, reason):
        with self.lock:
            if self.manifest and self.manifest['status'] == 'active':
                self.manifest.update(status='interrupted', finished=utc(), reason=reason)
                self.manifest['uncertainties'].append(reason)
                self.save()
                self.event('mission_interrupted', reason=reason)
            self.previews.clear()
            self.pending_goals.clear()
