import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'supervised'))
from metric_map import MetricGrid


def grid():
    cells = [0] * (40 * 40)
    for row in range(13, 27):
        cells[row * 40 + 28] = 100
    cells[24 * 40 + 15] = -1
    return MetricGrid({'source': 'ros-occupancy-grid', 'frame_id': 'map',
                       'width': 40, 'height': 40, 'resolution_m': .1,
                       'origin': [-2, -2, 0], 'stamp': 100., 'data': cells})


class MetricMapTests(unittest.TestCase):
    def test_rotated_origin_roundtrip(self):
        source = {'source': 'ros-occupancy-grid', 'frame_id': 'map', 'width': 2,
                  'height': 2, 'resolution_m': .5, 'origin': [1, 2, math.pi / 2],
                  'stamp': 10., 'data': [0, 10, 20, 100]}
        m = MetricGrid(source)
        for row in range(2):
            for col in range(2):
                self.assertEqual(m.world_to_cell(*m.cell_to_world(col, row)), (col, row))

    def test_measured_source_required(self):
        with self.assertRaises(ValueError):
            MetricGrid({'source': 'room-graph', 'frame_id': 'map'})

    def test_region_preserves_unknown_and_occupied(self):
        m = grid()
        region = m.region(-.5, .4, .4, .4)
        self.assertIn(-1, [v for row in region['cells'] for v in row])
        self.assertEqual(region['source'], 'ld06-slam')

    def test_swept_footprint_blocks_side_obstacle_and_unknown(self):
        m = grid()
        body = {'length_m': .5, 'width_m': .4, 'margin_m': .05}
        start = {'x': 0., 'y': 0., 'yaw': 0.}
        clear = m.preview(start, [{'x': .3, 'y': 0., 'yaw': 0.}], body)
        self.assertEqual(clear['verdict'], 'clear')
        blocked = m.preview(start, [{'x': 1., 'y': 0., 'yaw': 0.}], body)
        self.assertEqual(blocked['verdict'], 'blocked')
        unknown = m.preview(start, [{'x': -.5, 'y': .4, 'yaw': 0.}], body)
        self.assertEqual(unknown['verdict'], 'unknown')


if __name__ == '__main__':
    unittest.main()
