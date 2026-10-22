import tempfile
from pathlib import Path
import unittest
from supervised.topological_nav import TopologicalMap
from supervised.map_renderer import render_floor_map


class MapRendererTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.map_file = Path(self.temp_dir.name) / "test_graph.json"
        self.png_file = Path(self.temp_dir.name) / "test_floor_map.png"
        self.tmap = TopologicalMap(filepath=str(self.map_file))

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_render_empty_map(self):
        out = render_floor_map(self.tmap, output_path=str(self.png_file))
        self.assertTrue(Path(out).exists())
        self.assertGreater(Path(out).stat().st_size, 1000)

    def test_render_multiroom_with_objects_and_doors(self):
        self.tmap.add_room("living_room", center_relative_m=[0.0, 0.0])
        self.tmap.add_room("library", center_relative_m=[4.5, 0.0])
        self.tmap.current_room = "living_room"

        self.tmap.add_object("living_room", "chair", heading_deg=315.0, distance_m=1.5, expected_width_m=0.55)
        self.tmap.add_object("living_room", "chair", heading_deg=45.0, distance_m=1.8, expected_width_m=0.55)
        self.tmap.add_object("library", "sofa", heading_deg=120.0, distance_m=2.4, expected_width_m=1.8)

        self.tmap.add_doorway("door_1", "living_room", "library", heading_deg=45.0, width_m=0.85, is_passable=True)

        out = render_floor_map(self.tmap, output_path=str(self.png_file))
        self.assertTrue(Path(out).exists())
        self.assertGreater(Path(out).stat().st_size, 5000)


if __name__ == "__main__":
    unittest.main()
