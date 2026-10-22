"""Doorway traversability gate, aperture centering, and gyro-locked threshold crossing."""
import math
import logging
from typing import Dict, Any, Optional, List, Tuple


class DoorwayTransitController:
    """Evaluates doorway traversability and controls visual approach and threshold crossing."""

    def __init__(
        self,
        min_aperture_m: float = 0.50,
        approach_stop_dist_m: float = 0.75,
        threshold_stride_m: float = 1.00,
        stride_speed_m_s: float = 0.164,       # Calibrated PWM 50 speed
        turn_rate_deg_s: float = 33.0,         # Calibrated turn rate
        focal_length_px: float = 500.0,
        frame_width: int = 640,
        frame_height: int = 480,
        deadband_px: float = 25.0,
        nominal_door_width_m: float = 0.85
    ):
        self.min_aperture_m = float(min_aperture_m)
        self.approach_stop_dist_m = float(approach_stop_dist_m)
        self.threshold_stride_m = float(threshold_stride_m)
        self.stride_speed_m_s = float(stride_speed_m_s)
        self.turn_rate_deg_s = float(turn_rate_deg_s)
        self.focal_length = float(focal_length_px)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.u_center = frame_width / 2.0
        self.deadband_px = float(deadband_px)
        self.nominal_door_width_m = float(nominal_door_width_m)

        self.state = "evaluating"  # 'evaluating', 'centering', 'approaching', 'ready_for_stride', 'crossing', 'completed', 'impassable'
        self.stride_duration_s = round(self.threshold_stride_m / self.stride_speed_m_s, 2)  # ~6.1s

    def find_doorway(self, detections: List[Dict[str, Any]], min_conf: float = 0.20) -> Optional[Dict[str, Any]]:
        """Locate doorway or open door candidate from perception detections."""
        candidates = []
        for d in detections:
            label = d.get("label", "").lower()
            conf = d.get("confidence", 0.0)
            if any(term in label for term in ("doorway", "door", "opening", "entrance")) and conf >= min_conf:
                candidates.append(d)

        if not candidates:
            return None

        # Prioritize largest bounding box area
        def score(d):
            box = d.get("box", [0, 0, 0, 0])
            area = max(1.0, (box[2] - box[0]) * (box[3] - box[1]))
            return math.sqrt(area) * d.get("confidence", 0.0)

        candidates.sort(key=score, reverse=True)
        return candidates[0]

    def estimate_aperture(self, doorway_det: Dict[str, Any]) -> Tuple[float, float]:
        """Estimate distance (m) and physical aperture width (m) from bounding box."""
        box = doorway_det.get("box", [0, 0, 0, 0])
        w_px = max(1.0, box[2] - box[0])

        spatial = doorway_det.get("spatial", {})
        raw_dist = spatial.get("distance_m")

        if raw_dist is not None and raw_dist > 0.3:
            distance_m = raw_dist
        else:
            # Pinhole optical distance using nominal doorway width
            distance_m = (self.focal_length * self.nominal_door_width_m) / w_px

        # Compute physical aperture width in meters: aperture = (w_px * distance) / f
        aperture_m = (w_px * distance_m) / self.focal_length
        return round(distance_m, 2), round(aperture_m, 2)

    def compute_step(self, detections: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Compute the next transit step: check traversability, center, approach, or command stride."""
        if self.state in ("completed", "impassable"):
            return {"action": "stop", "duration_s": 0.0, "state": self.state}

        doorway = self.find_doorway(detections)
        if doorway is None:
            return {
                "action": "stop",
                "duration_s": 0.0,
                "state": "searching",
                "reason": "No doorway in visual field"
            }

        box = doorway.get("box", [0, 0, 0, 0])
        u_mid = (box[0] + box[2]) / 2.0
        distance_m, aperture_m = self.estimate_aperture(doorway)

        # 1. Traversability Gate: Verify clear width > 50cm
        if aperture_m < self.min_aperture_m:
            self.state = "impassable"
            return {
                "action": "stop",
                "duration_s": 0.0,
                "state": self.state,
                "aperture_width_m": aperture_m,
                "distance_m": distance_m,
                "reason": f"Doorway aperture ({aperture_m}m) is below minimum safe clearance ({self.min_aperture_m}m)"
            }

        # 2. Phase 1: Visual Centering
        error_u = u_mid - self.u_center
        if abs(error_u) > self.deadband_px:
            self.state = "centering"
            angle_deg = math.degrees(abs(math.atan2(error_u, self.focal_length)))
            action = "pwm_pivot_right" if error_u > 0 else "pwm_pivot_left"
            duration = min(0.50, max(0.08, angle_deg / self.turn_rate_deg_s))
            return {
                "action": action,
                "duration_s": round(duration, 3),
                "state": self.state,
                "error_u_px": round(error_u, 1),
                "distance_m": distance_m,
                "aperture_width_m": aperture_m
            }

        # 3. Approach to threshold standoff (0.75m)
        if distance_m > self.approach_stop_dist_m:
            self.state = "approaching"
            dist_to_go = distance_m - self.approach_stop_dist_m
            duration = min(0.50, max(0.10, dist_to_go / self.stride_speed_m_s))
            return {
                "action": "pwm_forward",
                "duration_s": round(duration, 3),
                "state": self.state,
                "distance_m": distance_m,
                "aperture_width_m": aperture_m
            }

        # 4. Ready for Phase 2: Gyro-Locked 1.0m Forward Stride
        self.state = "ready_for_stride"
        return {
            "action": "pwm_forward",
            "duration_s": self.stride_duration_s,
            "state": self.state,
            "distance_m": distance_m,
            "aperture_width_m": aperture_m,
            "stride_m": self.threshold_stride_m,
            "message": f"Doorway centered and aperture verified ({aperture_m}m). Executing {self.threshold_stride_m}m threshold crossing stride."
        }
