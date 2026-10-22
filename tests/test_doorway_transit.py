import unittest
from supervised.doorway_transit import DoorwayTransitController


class DoorwayTransitTests(unittest.TestCase):
    def setUp(self):
        self.controller = DoorwayTransitController(
            min_aperture_m=0.50,
            approach_stop_dist_m=0.75,
            threshold_stride_m=1.00,
            focal_length_px=500.0,
            nominal_door_width_m=0.85
        )

    def test_impassable_narrow_doorway_rejected(self):
        # Door cracked open only 30cm: box width is narrow at 1.5m distance
        # w_px = (f * aperture) / dist = (500 * 0.30) / 1.5 = 100px
        narrow_door = {
            "label": "doorway",
            "confidence": 0.85,
            "box": [270, 50, 370, 450],  # w_px = 100
            "spatial": {"distance_m": 1.50}
        }
        step = self.controller.compute_step([narrow_door])
        self.assertEqual(step["state"], "impassable")
        self.assertEqual(step["action"], "stop")
        self.assertLess(step["aperture_width_m"], 0.50)

    def test_centering_when_doorway_off_center(self):
        # Open door (80cm) to the left: u_mid = 150 < 320
        # w_px = (500 * 0.80) / 1.5 = 267px
        left_door = {
            "label": "doorway",
            "confidence": 0.90,
            "box": [17, 50, 283, 450],  # u_mid = 150
            "spatial": {"distance_m": 1.50}
        }
        step = self.controller.compute_step([left_door])
        self.assertEqual(step["state"], "centering")
        self.assertEqual(step["action"], "pwm_pivot_left")

    def test_approaching_when_centered_at_distance(self):
        # Open door (80cm) centered at 1.5m: u_mid = 320
        centered_door = {
            "label": "doorway",
            "confidence": 0.90,
            "box": [187, 50, 453, 450],  # u_mid = 320, w = 266px
            "spatial": {"distance_m": 1.50}
        }
        step = self.controller.compute_step([centered_door])
        self.assertEqual(step["state"], "approaching")
        self.assertEqual(step["action"], "pwm_forward")
        self.assertAlmostEqual(step["distance_m"], 1.50)

    def test_ready_for_stride_at_threshold_standoff(self):
        # Centered at standoff distance (0.75m): triggers 1.0m forward stride
        # w_px = (500 * 0.80) / 0.75 = 533px
        close_door = {
            "label": "doorway",
            "confidence": 0.92,
            "box": [54, 30, 586, 470],  # u_mid = 320, w = 532px
            "spatial": {"distance_m": 0.75}
        }
        step = self.controller.compute_step([close_door])
        self.assertEqual(step["state"], "ready_for_stride")
        self.assertEqual(step["action"], "pwm_forward")
        self.assertAlmostEqual(step["duration_s"], 6.10, places=1)
        self.assertEqual(step["stride_m"], 1.00)


if __name__ == "__main__":
    unittest.main()
