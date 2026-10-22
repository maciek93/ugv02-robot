import tempfile
from pathlib import Path
import unittest
import json
from supervised.topological_nav import TopologicalMap


class RoomGraphSpatialTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.map_file = Path(self.temp_dir.name) / "test_room_graph_v2.json"
        self.tmap = TopologicalMap(filepath=str(self.map_file))

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_add_and_query_objects(self):
        self.tmap.add_room("living_room")
        # Add chair_1 at heading 315 deg (left), distance 1.5m
        obj1 = self.tmap.add_object(
            room_name="living_room",
            label="chair",
            heading_deg=315.0,
            distance_m=1.5,
            expected_width_m=0.55
        )
        self.assertEqual(obj1["id"], "chair_1")
        self.assertEqual(obj1["horizontal_sector"], "left")
        self.assertEqual(obj1["distance_m"], 1.5)
        self.assertAlmostEqual(obj1["coords_m"][0], -1.06, places=2)  # 1.5 * sin(315)
        self.assertAlmostEqual(obj1["coords_m"][1], 1.06, places=2)   # 1.5 * cos(315)

        # Add chair_2 at heading 45 deg (right), distance 2.0m
        obj2 = self.tmap.add_object(
            room_name="living_room",
            label="chair",
            heading_deg=45.0,
            distance_m=2.0,
            expected_width_m=0.55
        )
        self.assertEqual(obj2["id"], "chair_2")
        self.assertEqual(obj2["horizontal_sector"], "right")

        # Query object
        matches = self.tmap.find_object("chair")
        self.assertEqual(len(matches), 2)
        self.assertEqual(matches[0]["room"], "living_room")
        self.assertEqual(matches[0]["object"]["id"], "chair_1")
        self.assertEqual(matches[1]["object"]["id"], "chair_2")

    def test_update_and_delete_object(self):
        self.tmap.add_room("library")
        self.tmap.add_object(
            room_name="library",
            label="sofa",
            heading_deg=120.0,
            distance_m=2.40,
            expected_width_m=1.80
        )
        self.assertIn("sofa", self.tmap.rooms["library"]["landmarks"])

        # Update sofa heading to 135.0 deg
        updated = self.tmap.update_object("library", "sofa_1", {"heading_deg": 135.0})
        self.assertTrue(updated)
        objs = self.tmap.get_room_objects("library")
        self.assertEqual(objs[0]["heading_deg"], 135.0)

        # Delete sofa
        deleted = self.tmap.delete_object("library", "sofa_1")
        self.assertTrue(deleted)
        self.assertEqual(len(self.tmap.get_room_objects("library")), 0)
        self.assertNotIn("sofa", self.tmap.rooms["library"]["landmarks"])

    def test_doorway_management_and_passability(self):
        self.tmap.add_room("hallway")
        self.tmap.add_room("bedroom")
        self.tmap.add_doorway("door_hb", "hallway", "bedroom", heading_deg=90.0, width_m=0.78)

        doorway = self.tmap.doorways[0]
        self.assertEqual(doorway["aperture_width_m"], 0.78)
        self.assertTrue(doorway["is_passable"])

        # Set doorway blocked
        self.tmap.set_doorway_passable("door_hb", False)
        self.assertFalse(self.tmap.doorways[0]["is_passable"])

        # Delete doorway
        deleted = self.tmap.delete_doorway("door_hb")
        self.assertTrue(deleted)
        self.assertEqual(len(self.tmap.doorways), 0)

    def test_backward_compatibility_v1(self):
        # Create a v1 format JSON file
        v1_data = {
            "version": 1,
            "current_room": "kitchen",
            "rooms": {
                "kitchen": {
                    "name": "kitchen",
                    "landmarks": ["refrigerator", "sink"],
                    "embeddings": []
                }
            },
            "doorways": [
                {
                    "id": "door_1",
                    "from": "kitchen",
                    "to": "living_room",
                    "heading_deg": 180.0,
                    "width_m": 0.85
                }
            ]
        }
        self.map_file.write_text(json.dumps(v1_data))

        # Load in TopologicalMap
        reloaded = TopologicalMap(filepath=str(self.map_file))
        self.assertIn("kitchen", reloaded.rooms)
        self.assertIn("objects", reloaded.rooms["kitchen"])
        self.assertEqual(reloaded.doorways[0]["aperture_width_m"], 0.85)
        self.assertTrue(reloaded.doorways[0]["is_passable"])

        # Save updates to v2
        reloaded.save()
        saved_data = json.loads(self.map_file.read_text())
        self.assertEqual(saved_data["version"], 2)


if __name__ == "__main__":
    unittest.main()
