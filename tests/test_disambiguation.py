import unittest
from supervised.servoing import VisualServoingController


class DisambiguationTests(unittest.TestCase):
    def setUp(self):
        # Two chairs side-by-side: left chair at u~160, right chair at u~480
        self.left_chair = {
            "label": "chair",
            "confidence": 0.85,
            "box": [110, 150, 210, 350],  # u_c = 160, area = 100 * 200 = 20000
            "spatial": {"distance_m": 1.8}
        }
        self.right_chair = {
            "label": "chair",
            "confidence": 0.85,
            "box": [430, 150, 530, 350],  # u_c = 480, area = 100 * 200 = 20000
            "spatial": {"distance_m": 1.8}
        }
        self.center_chair = {
            "label": "chair",
            "confidence": 0.80,
            "box": [270, 150, 370, 350],  # u_c = 320, centered
            "spatial": {"distance_m": 1.8}
        }

    def test_disambiguation_left_and_right(self):
        controller_left = VisualServoingController(target_label="chair", disambiguation="left")
        chosen_left = controller_left.find_target([self.right_chair, self.left_chair])
        self.assertEqual(chosen_left["box"], self.left_chair["box"])

        controller_right = VisualServoingController(target_label="chair", disambiguation="right")
        chosen_right = controller_right.find_target([self.left_chair, self.right_chair])
        self.assertEqual(chosen_right["box"], self.right_chair["box"])

    def test_disambiguation_center(self):
        controller_center = VisualServoingController(target_label="chair", disambiguation="center")
        chosen = controller_center.find_target([self.left_chair, self.right_chair, self.center_chair])
        self.assertEqual(chosen["box"], self.center_chair["box"])

    def test_disambiguation_by_index(self):
        # 1-based index from left to right: "1" is left, "2" is center, "3" is right
        candidates = [self.right_chair, self.center_chair, self.left_chair]
        c1 = VisualServoingController(target_label="chair", disambiguation="1").find_target(candidates)
        self.assertEqual(c1["box"], self.left_chair["box"])

        c2 = VisualServoingController(target_label="chair", disambiguation="2").find_target(candidates)
        self.assertEqual(c2["box"], self.center_chair["box"])

        c3 = VisualServoingController(target_label="chair", disambiguation="3").find_target(candidates)
        self.assertEqual(c3["box"], self.right_chair["box"])

    def test_target_persistence_locking(self):
        # Start by selecting the left chair
        controller = VisualServoingController(target_label="chair", disambiguation="left")
        first_chosen = controller.find_target([self.left_chair, self.right_chair])
        self.assertEqual(first_chosen["box"], self.left_chair["box"])

        # Frame 2: Both chairs shift slightly due to robot motion.
        # Even if the right chair becomes much larger/higher confidence, persistence must lock onto the left chair!
        shifted_left = {
            "label": "chair",
            "confidence": 0.70,
            "box": [115, 145, 215, 355],  # shifted slightly (IoU high with left_chair)
            "spatial": {"distance_m": 1.7}
        }
        giant_right = {
            "label": "chair",
            "confidence": 0.99,
            "box": [400, 100, 600, 450],  # huge box on right
            "spatial": {"distance_m": 1.2}
        }

        second_chosen = controller.find_target([giant_right, shifted_left])
        self.assertEqual(second_chosen["box"], shifted_left["box"],
                         "Controller should remain locked onto the left chair across consecutive frames")

    def test_memory_guided_turn(self):
        # Robot starts facing away from target, but knows target heading is 120 deg
        controller = VisualServoingController(
            target_label="sofa",
            stored_heading_deg=120.0
        )
        step = controller.compute_step([])  # No sofa in view initially
        self.assertEqual(step["state"], "memory_turn")
        self.assertEqual(step["action"], "pwm_spin_right")
        self.assertAlmostEqual(step["heading_deg"], 120.0)

        # After memory turn is executed, subsequent compute_step with no detection enters regular search
        step2 = controller.compute_step([])
        self.assertEqual(step2["state"], "searching")


if __name__ == "__main__":
    unittest.main()
