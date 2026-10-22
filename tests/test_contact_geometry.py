import math
import unittest
import numpy as np
from supervised.contact_geometry import ContactGeometry


class ContactGeometryTests(unittest.TestCase):
    def setUp(self):
        self.H = np.array([
            [0.0, -0.002, 0.96],
            [-0.002, 0.0, 0.64],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.K = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.geom = ContactGeometry(self.H, self.K, nominal_pitch_rad=0.0)

    def test_contact_pixel_without_mask(self):
        box = [200, 100, 400, 300]
        px = self.geom.compute_contact_pixel(box)
        self.assertEqual(px, [300.0, 300.0])

    def test_contact_pixel_with_mask(self):
        box = [200, 100, 400, 300]
        # Floor mask where walkable floor touches at v=305
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[295:306, 250:350] = 1
        px = self.geom.compute_contact_pixel(box, floor_mask=mask)
        self.assertEqual(px[1], 305.0)
        self.assertAlmostEqual(px[0], 300.0, delta=1.0)

    def test_homography_forward_and_inverse(self):
        # Pixel at (320, 240)
        ground_xy = self.geom.project_pixel_to_ground([320.0, 240.0])
        self.assertAlmostEqual(ground_xy[0], 0.48, places=4)
        self.assertAlmostEqual(ground_xy[1], 0.0, places=4)

        # Invert back to pixel
        reprojected_px = self.geom.project_ground_to_pixel(ground_xy)
        self.assertAlmostEqual(reprojected_px[0], 320.0, places=2)
        self.assertAlmostEqual(reprojected_px[1], 240.0, places=2)

    def test_pitch_delta_from_telemetry(self):
        # Stationary level robot
        telemetry = {'data': {'ax': 0, 'ay': 0, 'az': 8200}}
        delta = self.geom.pitch_delta_from_telemetry(telemetry)
        self.assertAlmostEqual(delta, 0.0, places=4)

        # Tilted 2 degrees nose up
        # ax = 8200 * sin(2 deg) ~= 286
        ax_2deg = 8200 * math.sin(math.radians(2.0))
        az_2deg = 8200 * math.cos(math.radians(2.0))
        telemetry_tilted = {'data': {'ax': ax_2deg, 'ay': 0, 'az': az_2deg}}
        delta_tilted = self.geom.pitch_delta_from_telemetry(telemetry_tilted)
        self.assertAlmostEqual(math.degrees(delta_tilted), 2.0, places=2)

    def test_pitch_compensation_invariance(self):
        # Nominal point at X=0.48, Y=0.0 -> pixel (320, 240)
        nominal_xy = self.geom.project_pixel_to_ground([320.0, 240.0], pitch_delta_rad=0.0)

        # Simulate 2 deg nose-up pitch tilt:
        # Camera ray tilts up by 2 deg -> ground point appears at shifted pixel
        d_theta = math.radians(2.0)
        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, math.cos(d_theta), -math.sin(d_theta)],
            [0.0, math.sin(d_theta), math.cos(d_theta)]
        ])
        ray_nom = np.linalg.inv(self.K) @ np.array([320.0, 240.0, 1.0])
        ray_tilted = Rx @ ray_nom
        px_tilted = self.K @ ray_tilted
        px_tilted = (px_tilted / px_tilted[2])[:2]

        # Using pitch compensation, the tilted pixel must evaluate to the same metric ground XY!
        compensated_xy = self.geom.project_pixel_to_ground(px_tilted, pitch_delta_rad=d_theta)
        self.assertAlmostEqual(compensated_xy[0], nominal_xy[0], places=3)
        self.assertAlmostEqual(compensated_xy[1], nominal_xy[1], places=3)

    def test_evaluate_object(self):
        box = [270, 190, 370, 240]
        res = self.geom.evaluate_object(box)
        self.assertIn('contact_px', res)
        self.assertIn('ground_xy_m', res)
        self.assertIn('distance_m', res)
        self.assertIn('bearing_deg', res)
        self.assertAlmostEqual(res['contact_px'][0], 320.0)
        self.assertAlmostEqual(res['contact_px'][1], 240.0)
        self.assertAlmostEqual(res['ground_xy_m'][0], 0.48, places=2)
        self.assertAlmostEqual(res['ground_xy_m'][1], 0.0, places=2)


if __name__ == '__main__':
    unittest.main()
