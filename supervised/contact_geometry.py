"""Metric ground-plane contact geometry with dynamic IMU pitch compensation.

Coordinates: metres, X forward / Y left (positive yaw is left).
Image coordinates: pixels, u right / v down.
"""
import math
import cv2
import numpy as np


class ContactGeometry:
    def __init__(self, image_to_ground, camera_matrix=None, nominal_pitch_rad=0.0):
        self.H_nominal = np.asarray(image_to_ground, dtype=np.float64)
        if self.H_nominal.shape != (3, 3):
            raise ValueError("image_to_ground must be a 3x3 matrix")
        self.K = np.asarray(camera_matrix, dtype=np.float64) if camera_matrix is not None else None
        if self.K is not None and self.K.shape != (3, 3):
            raise ValueError("camera_matrix must be a 3x3 matrix")
        self.K_inv = np.linalg.inv(self.K) if self.K is not None else None
        self.nominal_pitch = float(nominal_pitch_rad)

    def pitch_from_imu(self, ax, ay, az):
        """Compute pitch angle in radians from accelerometer readings."""
        denom = math.sqrt(ay * ay + az * az)
        if denom < 1e-6:
            return 0.0
        return math.atan2(ax, denom)

    def pitch_delta_from_telemetry(self, chassis_data):
        """Compute pitch delta (radians) relative to nominal from chassis telemetry data."""
        if not chassis_data or not isinstance(chassis_data, dict):
            return 0.0
        data = chassis_data.get('data') if 'data' in chassis_data else chassis_data
        if not isinstance(data, dict):
            return 0.0
        ax = data.get('ax')
        ay = data.get('ay')
        az = data.get('az')
        if ax is None or ay is None or az is None:
            return 0.0
        try:
            ax, ay, az = float(ax), float(ay), float(az)
        except (ValueError, TypeError):
            return 0.0
        # Check that total acceleration magnitude is reasonable (stationary / steady drive)
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        if mag < 1000 or mag > 20000:  # Nominal 1g is ~8200 LSB
            return 0.0
        pitch = self.pitch_from_imu(ax, ay, az)
        delta = pitch - self.nominal_pitch
        # Clamp pitch delta to physical chassis suspension/tilt limits (+- 10 degrees)
        max_delta = math.radians(10.0)
        return float(np.clip(delta, -max_delta, max_delta))

    def compensated_homography(self, pitch_delta_rad=0.0):
        """Apply rotational pitch compensation to nominal ground homography."""
        if self.K is None or abs(pitch_delta_rad) < 1e-5:
            return self.H_nominal
        cos_p = math.cos(pitch_delta_rad)
        sin_p = math.sin(pitch_delta_rad)
        # Rx for pitch tilt in camera frame
        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cos_p, -sin_p],
            [0.0, sin_p, cos_p]
        ], dtype=np.float64)
        # H_live = H_nominal @ K @ Rx.T @ K_inv
        H_live = self.H_nominal @ self.K @ Rx.T @ self.K_inv
        return H_live

    def compute_contact_pixel(self, box, floor_mask=None):
        """Find the ground contact pixel [u, v] for a bounding box.
        
        Args:
            box: [x1, y1, x2, y2] in pixels.
            floor_mask: Optional 2D array (H, W) where >0 represents walkable floor.
        Returns:
            [u, v] as float coordinates.
        """
        x1, y1, x2, y2 = [float(v) for v in box]
        u_center = (x1 + x2) / 2.0
        v_base = y2

        if floor_mask is None:
            return [u_center, v_base]

        h, w = floor_mask.shape[:2]
        # Restrict horizontal search to inner 60% of bounding box
        span = x2 - x1
        u_start = max(0, int(round(x1 + 0.20 * span)))
        u_end = min(w - 1, int(round(x2 - 0.20 * span)))
        if u_end <= u_start:
            u_start, u_end = max(0, int(round(x1))), min(w - 1, int(round(x2)))

        # Search vertical window around bottom edge (+- 12 pixels)
        v_start = max(0, int(round(y2 - 12)))
        v_end = min(h - 1, int(round(y2 + 12)))

        sub = floor_mask[v_start:v_end + 1, u_start:u_end + 1]
        mask_pts = np.argwhere(sub > 0)
        if len(mask_pts) > 0:
            # Pick the lowest point (largest v) in the contact region
            best_rel_v = int(np.max(mask_pts[:, 0]))
            v_contact = v_start + best_rel_v
            # Get median u at that best v
            u_indices = mask_pts[mask_pts[:, 0] == best_rel_v, 1]
            u_contact = u_start + float(np.median(u_indices))
            return [u_contact, float(v_contact)]

        return [u_center, v_base]

    def project_pixel_to_ground(self, pixel_uv, pitch_delta_rad=0.0):
        """Project image pixel [u, v] to metric chassis ground coordinates [X, Y]."""
        H = self.compensated_homography(pitch_delta_rad)
        pts = np.array([[[float(pixel_uv[0]), float(pixel_uv[1])]]], dtype=np.float32)
        res = cv2.perspectiveTransform(pts, H)
        xy = res[0, 0]
        return [float(xy[0]), float(xy[1])]

    def project_ground_to_pixel(self, ground_xy, pitch_delta_rad=0.0):
        """Project metric ground coordinates [X, Y] back to image pixel [u, v]."""
        H = self.compensated_homography(pitch_delta_rad)
        H_inv = np.linalg.inv(H)
        pts = np.array([[[float(ground_xy[0]), float(ground_xy[1])]]], dtype=np.float32)
        res = cv2.perspectiveTransform(pts, H_inv)
        uv = res[0, 0]
        return [float(uv[0]), float(uv[1])]

    def evaluate_object(self, box, floor_mask=None, pitch_delta_rad=0.0):
        """Compute full metric spatial state for a detected bounding box."""
        contact_px = self.compute_contact_pixel(box, floor_mask)
        ground_xy = self.project_pixel_to_ground(contact_px, pitch_delta_rad)
        x_m, y_m = ground_xy
        dist_m = math.hypot(x_m, y_m)
        bearing_rad = math.atan2(y_m, x_m)
        bearing_deg = math.degrees(bearing_rad)
        w_px = float(box[2] - box[0])
        h_px = float(box[3] - box[1])

        return {
            'contact_px': contact_px,
            'ground_xy_m': [round(x_m, 4), round(y_m, 4)],
            'distance_m': round(dist_m, 4),
            'bearing_deg': round(bearing_deg, 2),
            'bearing_rad': round(bearing_rad, 4),
            'box_width_px': round(w_px, 1),
            'box_height_px': round(h_px, 1)
        }
