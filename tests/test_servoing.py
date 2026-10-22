import unittest
from supervised.servoing import VisualServoingController


class VisualServoingTests(unittest.TestCase):
    def setUp(self):
        self.controller = VisualServoingController(
            target_label="chair",
            target_distance_m=0.50,
            frame_width=640,
            frame_height=480,
            focal_length_px=500.0,
            deadband_px=35.0
        )

    def test_search_when_target_missing(self):
        step = self.controller.compute_step([])
        self.assertEqual(step["state"], "searching")
        self.assertEqual(step["action"], "pwm_spin_right")
        self.assertEqual(step["duration_s"], self.controller.search_step_s)

    def test_target_lost_after_360_scan(self):
        # Accumulate 360 deg
        for _ in range(60):
            step = self.controller.compute_step([])
            if step["state"] == "lost":
                break
        self.assertEqual(step["state"], "lost")
        self.assertEqual(step["action"], "stop")

    def test_centering_when_target_to_the_right(self):
        # Target bounding box on the right side: [450, 200, 550, 400], center u=500
        detections = [{
            "label": "chair",
            "confidence": 0.85,
            "box": [450, 200, 550, 400],
            "spatial": {"distance_m": 1.5}
        }]
        step = self.controller.compute_step(detections)
        self.assertEqual(step["state"], "centering")
        self.assertEqual(step["action"], "pwm_pivot_right")
        self.assertGreater(step["error_u_px"], 35.0)
        self.assertLessEqual(step["duration_s"], self.controller.max_step_s)

    def test_centering_when_target_to_the_left(self):
        # Target bounding box on the left side: [100, 200, 200, 400], center u=150
        detections = [{
            "label": "chair",
            "confidence": 0.85,
            "box": [100, 200, 200, 400],
            "spatial": {"distance_m": 1.5}
        }]
        step = self.controller.compute_step(detections)
        self.assertEqual(step["state"], "centering")
        self.assertEqual(step["action"], "pwm_pivot_left")
        self.assertLess(step["error_u_px"], -35.0)
        self.assertLessEqual(step["duration_s"], self.controller.max_step_s)

    def test_approaching_when_centered(self):
        # Target bounding box centered: [270, 200, 370, 400], center u=320
        detections = [{
            "label": "chair",
            "confidence": 0.85,
            "box": [270, 200, 370, 400],
            "spatial": {"distance_m": 1.20}
        }]
        step = self.controller.compute_step(detections)
        self.assertEqual(step["state"], "approaching")
        self.assertEqual(step["action"], "pwm_forward")
        self.assertLessEqual(step["duration_s"], self.controller.max_step_s)
        self.assertAlmostEqual(step["distance_m"], 1.20)

    def test_arrived_when_distance_reached(self):
        # Target within stopping distance
        detections = [{
            "label": "chair",
            "confidence": 0.90,
            "box": [270, 200, 370, 400],
            "spatial": {"distance_m": 0.48}
        }]
        step = self.controller.compute_step(detections)
        self.assertEqual(step["state"], "arrived")
        self.assertEqual(step["action"], "stop")
        self.assertEqual(step["duration_s"], 0.0)

    def test_safety_stop_if_target_lost_during_approach(self):
        self.controller.max_lost_frames = 2
        detections = [{
            "label": "chair",
            "confidence": 0.85,
            "box": [270, 200, 370, 400],
            "spatial": {"distance_m": 1.20}
        }]
        step1 = self.controller.compute_step(detections)
        self.assertEqual(step1["state"], "approaching")

        # Lost for 1 frame: holds
        step2 = self.controller.compute_step([])
        self.assertEqual(step2["action"], "stop")

        # Lost for 2nd frame: declares lost and halts
        step3 = self.controller.compute_step([])
        self.assertEqual(step3["state"], "lost")
        self.assertEqual(step3["action"], "stop")

    def test_dynamic_expected_width(self):
        controller = VisualServoingController(
            target_label="backpack",
            target_distance_m=0.50,
            expected_width_m=0.35,
            focal_length_px=500.0
        )
        detections = [{
            "label": "backpack",
            "confidence": 0.90,
            "box": [270, 200, 370, 400],
            "spatial": {"distance_m": None}
        }]
        step = controller.compute_step(detections)
        self.assertAlmostEqual(step["distance_m"], 1.75, places=2)


if __name__ == '__main__':
    unittest.main()
