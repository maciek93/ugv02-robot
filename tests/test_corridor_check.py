import unittest
from supervised.corridor_check import CorridorObstacleChecker


class CorridorObstacleCheckerTests(unittest.TestCase):
    def setUp(self):
        self.checker = CorridorObstacleChecker(
            min_clear_x_m=0.15,
            max_clear_x_m=0.70,
            corridor_half_width_m=0.22,
            retreat_dist_m=0.12,
            bypass_turn_deg=25.0
        )

    def test_corridor_clear_when_no_obstacles(self):
        blocked, obs = self.checker.check_corridor([])
        self.assertFalse(blocked)
        self.assertIsNone(obs)

    def test_target_object_itself_does_not_trigger_reflex(self):
        # The sofa being pursued at 0.50m directly ahead is the target, not an obstacle!
        target_det = {
            "label": "sofa",
            "confidence": 0.90,
            "box": [200, 200, 440, 450],
            "spatial": {"distance_m": 0.50, "ground_xy_m": [0.50, 0.0]}
        }
        blocked, obs = self.checker.check_corridor([target_det], target_label="sofa")
        self.assertFalse(blocked)

    def test_unexpected_obstacle_blocks_corridor(self):
        # A shoe or cardboard box on the floor 0.40m ahead, right in track path
        shoe_det = {
            "label": "shoe",
            "confidence": 0.88,
            "box": [280, 390, 360, 470],
            "spatial": {"distance_m": 0.40, "ground_xy_m": [0.40, 0.05]}
        }
        blocked, obs = self.checker.check_corridor([shoe_det], target_label="sofa")
        self.assertTrue(blocked)
        self.assertEqual(obs["label"], "shoe")

    def test_reflex_execution_sequence(self):
        obstacle = {
            "label": "backpack",
            "box": [350, 400, 450, 470],  # u_c = 400 (to right of center)
            "spatial": {"distance_m": 0.35, "ground_xy_m": [0.35, 0.15]}
        }
        self.checker.start_reflex(obstacle)
        self.assertTrue(self.checker.reflex_active)
        self.assertEqual(self.checker.preferred_bypass_dir, "left")

        # Step 1: Halt
        s1 = self.checker.compute_reflex_step()
        self.assertEqual(s1["state"], "reflex_halt")

        # Step 2: Retreat
        s2 = self.checker.compute_reflex_step()
        self.assertEqual(s2["state"], "reflex_retreat")

        # Step 3: Pivot left
        s3 = self.checker.compute_reflex_step()
        self.assertEqual(s3["state"], "reflex_pivot")
        self.assertEqual(s3["action"], "pwm_pivot_left")

        # Step 4: Advance
        s4 = self.checker.compute_reflex_step()
        self.assertEqual(s4["state"], "reflex_advance")

        # Step 5: Complete
        s5 = self.checker.compute_reflex_step()
        self.assertEqual(s5["state"], "reflex_complete")
        self.assertFalse(self.checker.reflex_active)


if __name__ == "__main__":
    unittest.main()
