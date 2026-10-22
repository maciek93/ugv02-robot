import tempfile
from pathlib import Path
import unittest
import numpy as np
from supervised.topological_nav import TopologicalMap, SceneFeatureExtractor


class TopologicalNavTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.map_file = Path(self.temp_dir.name) / "test_room_graph.json"
        self.tmap = TopologicalMap(filepath=str(self.map_file))

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_add_and_rename_room(self):
        self.tmap.add_room("Hallway", landmarks=["doorway", "radiator"])
        self.assertIn("hallway", self.tmap.rooms)
        self.assertEqual(self.tmap.rooms["hallway"]["type"], "corridor")

        self.tmap.add_room("Kitchen", landmarks=["sink", "oven"])
        self.tmap.add_doorway("door_hk", "hallway", "kitchen", heading_deg=45.0)

        # Rename kitchen to pantry
        self.tmap.rename_room("kitchen", "pantry")
        self.assertNotIn("kitchen", self.tmap.rooms)
        self.assertIn("pantry", self.tmap.rooms)
        self.assertEqual(self.tmap.doorways[0]["to"], "pantry")

    def test_find_path(self):
        self.tmap.add_room("hallway")
        self.tmap.add_room("kitchen")
        self.tmap.add_room("dining_room")
        self.tmap.add_room("bedroom")  # isolated

        self.tmap.add_doorway("door_hk", "hallway", "kitchen")
        self.tmap.add_doorway("door_kd", "kitchen", "dining_room")

        # Path from hallway to dining_room
        path = self.tmap.find_path("hallway", "dining_room")
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 2)
        self.assertEqual(path[0]["from"], "hallway")
        self.assertEqual(path[0]["to"], "kitchen")
        self.assertEqual(path[1]["from"], "kitchen")
        self.assertEqual(path[1]["to"], "dining_room")

        # Same room
        self.assertEqual(self.tmap.find_path("hallway", "hallway"), [])

        # Disconnected room
        self.assertIsNone(self.tmap.find_path("hallway", "bedroom"))

    def test_suggest_room_name(self):
        # Kitchen landmarks
        name = self.tmap.suggest_room_name(["refrigerator", "sink"])
        self.assertEqual(name, "kitchen")

        # Living room landmarks
        name = self.tmap.suggest_room_name(["couch", "television"])
        self.assertEqual(name, "living_room")

        # Unnamed fallback
        name = self.tmap.suggest_room_name(["mysterious_box"])
        self.assertEqual(name, "unnamed_room_1")

    def test_novelty_detection(self):
        img_room1 = np.full((480, 640, 3), 100, dtype=np.uint8)
        img_room1[100:300, 100:300] = [200, 50, 50]  # distinct feature

        emb1 = self.tmap.compute_embedding(img_room1)
        self.tmap.add_room("room_1", embeddings=[emb1.tolist()])

        # Similar image: not novel
        img_similar = img_room1.copy()
        img_similar[105:305, 105:305] = [195, 55, 55]
        is_novel, sim, closest = self.tmap.check_novelty(img_similar, threshold=0.65)
        self.assertFalse(is_novel)
        self.assertEqual(closest, "room_1")
        self.assertGreaterEqual(sim, 0.65)

        # Completely different image: novel
        img_novel = np.zeros((480, 640, 3), dtype=np.uint8)
        img_novel[:, :, 1] = 255  # bright green
        is_novel, sim, _ = self.tmap.check_novelty(img_novel, threshold=0.65)
        self.assertTrue(is_novel)
        self.assertLess(sim, 0.65)

    def test_persistence(self):
        self.tmap.add_room("lab", landmarks=["microscope"])
        self.tmap.current_room = "lab"
        self.tmap.save()

        # Reload in a new instance
        reloaded = TopologicalMap(filepath=str(self.map_file))
        self.assertIn("lab", reloaded.rooms)
        self.assertEqual(reloaded.current_room, "lab")
        self.assertEqual(reloaded.rooms["lab"]["landmarks"], ["microscope"])


if __name__ == '__main__':
    unittest.main()
