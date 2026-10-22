from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'supervised'))
from metric_tools import validate_route_scope


MISSION = {'rooms': ['kitchen', 'hall'],
           'doorways': [{'id': 'kitchen-hall', 'from_room': 'kitchen', 'to_room': 'hall'}]}


class MetricScopeTests(unittest.TestCase):
    def test_directed_doorway_is_required_for_a_room_change(self):
        destination, edges = validate_route_scope(MISSION, 'kitchen', [
            {'room': 'kitchen'}, {'room': 'hall', 'via_doorway': 'kitchen-hall'}])
        self.assertEqual(destination, 'hall')
        self.assertEqual(edges, MISSION['doorways'])
        with self.assertRaises(PermissionError):
            validate_route_scope(MISSION, 'hall', [{'room': 'kitchen', 'via_doorway': 'kitchen-hall'}])
        with self.assertRaises(PermissionError):
            validate_route_scope(MISSION, 'kitchen', [{'room': 'hall'}])


if __name__ == '__main__':
    unittest.main()
