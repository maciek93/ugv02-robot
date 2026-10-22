"""Measured ROS occupancy-grid inspection and route geometry.

This module never derives a metric map from the legacy room diagram.  A map must
carry a ROS OccupancyGrid source, resolution, origin and measured cell values.
"""
import hashlib
import json
import math


def finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


def angle_delta(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


class MetricGrid:
    def __init__(self, snapshot):
        if snapshot.get('source') != 'ros-occupancy-grid' or snapshot.get('frame_id') != 'map':
            raise ValueError('A measured ROS map-frame occupancy grid is required')
        self.width = snapshot['width']
        self.height = snapshot['height']
        if type(self.width) is not int or type(self.height) is not int or not 1 <= self.width <= 4096 or not 1 <= self.height <= 4096:
            raise ValueError('Invalid grid dimensions')
        self.resolution = finite(snapshot['resolution_m'], 'resolution_m')
        if not .01 <= self.resolution <= .5:
            raise ValueError('Invalid grid resolution')
        self.origin = tuple(finite(x, 'origin') for x in snapshot['origin'])
        if len(self.origin) != 3:
            raise ValueError('Origin must be x, y, yaw')
        self.data = snapshot['data']
        if len(self.data) != self.width * self.height or any(type(v) is not int or not -1 <= v <= 100 for v in self.data):
            raise ValueError('Invalid occupancy cells')
        self.stamp = finite(snapshot['stamp'], 'stamp')
        self.version = snapshot.get('map_version') or self._digest()
        self.calibration_status = snapshot.get('calibration_status', 'measured-profile')

    def _digest(self):
        payload = json.dumps([self.width, self.height, self.resolution, self.origin, self.data], separators=(',', ':'))
        return hashlib.sha256(payload.encode()).hexdigest()[:16]

    def world_to_cell(self, x, y):
        x, y = finite(x, 'x'), finite(y, 'y')
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(yaw), math.sin(yaw)
        return math.floor((c * dx + s * dy) / self.resolution), math.floor((-s * dx + c * dy) / self.resolution)

    def cell_to_world(self, col, row):
        ox, oy, yaw = self.origin
        x, y = (col + .5) * self.resolution, (row + .5) * self.resolution
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c*x - s*y, oy + s*x + c*y

    def occupancy(self, col, row):
        if not 0 <= col < self.width or not 0 <= row < self.height:
            return -1
        return self.data[row * self.width + col]

    def region(self, center_x, center_y, width_m=4., height_m=4., max_cells=16384):
        width_m, height_m = finite(width_m, 'width_m'), finite(height_m, 'height_m')
        if not .1 <= width_m <= 20 or not .1 <= height_m <= 20:
            raise ValueError('Region dimensions must be 0.1–20 metres')
        cx, cy = self.world_to_cell(center_x, center_y)
        cols, rows = max(1, math.ceil(width_m / self.resolution)), max(1, math.ceil(height_m / self.resolution))
        if cols * rows > max_cells:
            raise ValueError('Region exceeds maximum cell count; request a smaller crop')
        left, bottom = cx - cols // 2, cy - rows // 2
        values = [[self.occupancy(left + x, bottom + y) for x in range(cols)] for y in range(rows)]
        return {'source': 'ld06-slam', 'frame_id': 'map', 'map_version': self.version,
                'calibration_status': self.calibration_status,
                'stamp': self.stamp, 'resolution_m': self.resolution,
                'bottom_left_cell': [left, bottom], 'cells': values,
                'legend': {'unknown': -1, 'free': '0..49', 'occupied': '50..100'}}

    def footprint(self, x, y, yaw, length_m, width_m, margin_m):
        length_m, width_m, margin_m = (finite(v, k) for v, k in ((length_m, 'length_m'), (width_m, 'width_m'), (margin_m, 'margin_m')))
        if not .05 <= length_m <= 2 or not .05 <= width_m <= 2 or not 0 <= margin_m <= 1:
            raise ValueError('Invalid chassis footprint')
        half_l, half_w = length_m / 2 + margin_m, width_m / 2 + margin_m
        step = self.resolution / 2
        c, s = math.cos(yaw), math.sin(yaw)
        occupied = unknown = 0
        worst = None
        seen = set()
        ix = math.ceil(2 * half_l / step)
        iy = math.ceil(2 * half_w / step)
        for a in range(ix + 1):
            forward = -half_l + 2 * half_l * a / ix
            for b in range(iy + 1):
                lateral = -half_w + 2 * half_w * b / iy
                col, row = self.world_to_cell(x + c*forward - s*lateral, y + s*forward + c*lateral)
                if (col, row) in seen:
                    continue
                seen.add((col, row))
                value = self.occupancy(col, row)
                if value < 0:
                    unknown += 1
                    worst = worst or {'cell': [col, row], 'occupancy': value}
                elif value >= 50:
                    occupied += 1
                    worst = {'cell': [col, row], 'occupancy': value}
        return {'occupied_cells': occupied, 'unknown_cells': unknown, 'first_issue': worst}

    def preview(self, start, waypoints, footprint, allow_unknown=False):
        if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 100:
            raise ValueError('Provide 1–100 waypoints')
        if type(allow_unknown) is not bool:
            raise ValueError('allow_unknown must be boolean')
        poses = [start, *waypoints]
        poses = [tuple(finite(p[k], k) for k in ('x', 'y', 'yaw')) for p in poses]
        total = 0.
        first_issue = None
        occupied = unknown = samples = 0
        for a, b in zip(poses, poses[1:]):
            distance = math.hypot(b[0] - a[0], b[1] - a[1])
            total += distance
            turns = abs(angle_delta(a[2], b[2])) * max(footprint['length_m'], footprint['width_m']) / 2
            count = max(1, math.ceil(max(distance, turns) / (self.resolution / 2)))
            for i in range(count + 1):
                t = i / count
                x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
                yaw = a[2] + t * angle_delta(a[2], b[2])
                check = self.footprint(x, y, yaw, **footprint)
                samples += 1
                occupied += check['occupied_cells']
                unknown += check['unknown_cells']
                if check['first_issue'] and not first_issue:
                    first_issue = {'pose': {'x': x, 'y': y, 'yaw': yaw}, **check['first_issue']}
        verdict = 'blocked' if occupied else 'unknown' if unknown and not allow_unknown else 'clear'
        return {'verdict': verdict, 'map_version': self.version, 'frame_id': 'map',
                'distance_m': round(total, 3), 'sampled_poses': samples,
                'occupied_samples': occupied, 'unknown_samples': unknown,
                'first_issue': first_issue, 'waypoints': waypoints}
