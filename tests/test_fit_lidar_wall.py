import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fit_lidar_wall import fit_wall


class WallFitTests(unittest.TestCase):
    def test_recovers_mount_yaw_from_a_side_wall(self):
        count, yaw = 720, .1
        angles = [i * 2 * math.pi / (count - 1) for i in range(count)]
        ranges = []
        for angle in angles:
            direction_y = math.sin(angle + yaw)
            distance = 1 / direction_y if direction_y > 0 else math.inf
            ranges.append(distance if .15 <= distance <= 4 else None)
        scan = {'sensor': 'LD06', 'frame_id': 'base_laser', 'angle_min': 0,
                'angle_increment': 2 * math.pi / (count - 1), 'ranges_m': ranges}
        result = fit_wall(scan, 'left')
        self.assertAlmostEqual(result['candidate_lidar_yaw_rad'], yaw, places=3)
        self.assertAlmostEqual(result['wall_distance_from_lidar_m'], 1, places=3)

    def test_recovers_front_wall_when_raw_zero_points_right(self):
        count, yaw = 720, math.radians(-86)
        angles = [i * 2 * math.pi / (count - 1) for i in range(count)]
        ranges = []
        for angle in angles:
            direction_x = math.cos(angle + yaw)
            distance = 1 / direction_x if direction_x > 0 else math.inf
            ranges.append(distance if .15 <= distance <= 4 else None)
        scan = {'sensor': 'LD06', 'frame_id': 'base_laser', 'angle_min': 0,
                'angle_increment': 2 * math.pi / (count - 1), 'ranges_m': ranges}
        result = fit_wall(scan, 'front', scan_sector='left')
        self.assertAlmostEqual(result['candidate_lidar_yaw_rad'], yaw, places=3)
        self.assertAlmostEqual(result['wall_distance_from_lidar_m'], 1, places=3)


if __name__ == '__main__':
    unittest.main()
