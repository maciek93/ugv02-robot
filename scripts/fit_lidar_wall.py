#!/usr/bin/env python3
"""Estimate LD06 mounting yaw from a scan while the chassis is parallel to a wall.

Save `scripts/metric.py view-lidar-scan` output to a JSON file first. Run with
`--side left`, `right`, or `front` describes the wall relative to the chassis.
`--scan-sector` selects where that wall appears in the raw LiDAR scan, which may
be rotated relative to the chassis. The result is a wall fit, not a completed
sensor calibration: wall alignment and mount offset still need a physical check.
"""
import argparse
import json
import math
from pathlib import Path
import random


def fit_wall(scan, side, tolerance=.035, scan_sector=None):
    if scan.get('sensor') != 'LD06' or scan.get('frame_id') != 'base_laser':
        raise ValueError('A sensor-labelled LD06 scan is required')
    scan_sector = scan_sector or side
    if scan_sector not in ('front', 'left', 'right'):
        raise ValueError('Unknown raw scan sector')
    points = []
    for i, distance in enumerate(scan['ranges_m']):
        if not isinstance(distance, (int, float)) or not .15 <= distance <= 4.:
            continue
        angle = scan['angle_min'] + i * scan['angle_increment']
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        # Restrict the fit to a wall approximately to the chosen side. The
        # assumed mounting error is small; omit forward and rear clutter.
        if scan_sector == 'front' and abs(angle) > math.radians(50):
            continue
        if scan_sector == 'left' and not math.radians(40) <= angle <= math.radians(140):
            continue
        if scan_sector == 'right' and not -math.radians(140) <= angle <= -math.radians(40):
            continue
        points.append((distance * math.cos(angle), distance * math.sin(angle)))
    if len(points) < 20:
        raise ValueError('Too few side-wall LiDAR points')
    rng = random.Random(0)
    best = []
    for _ in range(min(1000, len(points) * 15)):
        a, b = rng.sample(points, 2)
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < .15:
            continue
        nx, ny = -dy / length, dx / length
        inliers = [p for p in points if abs(nx * (p[0] - a[0]) + ny * (p[1] - a[1])) < tolerance]
        if len(inliers) > len(best):
            best = inliers
    if len(best) < 20 or len(best) / len(points) < .25:
        raise ValueError('No stable straight wall found in this scan')
    mean_x = sum(x for x, _ in best) / len(best)
    mean_y = sum(y for _, y in best) / len(best)
    sxx = sum((x - mean_x) ** 2 for x, _ in best)
    syy = sum((y - mean_y) ** 2 for _, y in best)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in best)
    wall_angle = .5 * math.atan2(2 * sxy, sxx - syy)
    wall_angle = (wall_angle + math.pi / 2) % math.pi - math.pi / 2
    nx, ny = -math.sin(wall_angle), math.cos(wall_angle)
    residual = math.sqrt(sum((nx * (x - mean_x) + ny * (y - mean_y)) ** 2
                             for x, y in best) / len(best))
    expected_wall_angle = math.pi / 2 if side == 'front' else 0.
    nominal_yaw = 0.
    if side == 'front' and scan_sector == 'left':
        nominal_yaw = -math.pi / 2
    elif side == 'front' and scan_sector == 'right':
        nominal_yaw = math.pi / 2
    candidate_yaw = nominal_yaw + (expected_wall_angle - wall_angle - nominal_yaw
                                   + math.pi / 2) % math.pi - math.pi / 2
    candidate_yaw = (candidate_yaw + math.pi) % (2 * math.pi) - math.pi
    return {'sensor': 'LD06', 'source_sequence': scan.get('sequence'),
            'source_stamp': scan.get('stamp'), 'side': side, 'scan_sector': scan_sector,
            'inliers': len(best), 'candidate_points': len(points),
            'wall_angle_in_lidar_rad': wall_angle,
            'candidate_lidar_yaw_rad': candidate_yaw,
            'candidate_lidar_yaw_deg': math.degrees(candidate_yaw),
            'wall_distance_from_lidar_m': abs(nx * mean_x + ny * mean_y),
            'fit_rms_m': residual,
            'assumption': 'chassis front edge physically parallel to this wall if side=front; chassis forward axis parallel to wall otherwise'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scan_json', type=Path)
    parser.add_argument('--side', choices=('left', 'right', 'front'), required=True)
    parser.add_argument('--scan-sector', choices=('left', 'right', 'front'))
    args = parser.parse_args()
    print(json.dumps(fit_wall(json.loads(args.scan_json.read_text()), args.side,
                              scan_sector=args.scan_sector), indent=2))
