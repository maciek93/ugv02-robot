"""Image-Based Visual Servoing (IBVS) for targeted object approach.

Controls chassis motion using calibrated PWM primitives to acquire,
center, approach, and halt smoothly at a specified metric distance
from a visual target (e.g. 'chair', 'doorway', 'table').
"""
import math
import time
import logging
from typing import Dict, Any, Optional, List


class VisualServoingController:
    """Computes closed-loop visual servoing actions to approach a target object."""

    def __init__(
        self,
        target_label: str = "chair",
        target_distance_m: float = 0.50,
        expected_width_m: Optional[float] = None,
        disambiguation: Optional[str] = None,
        stored_heading_deg: Optional[float] = None,
        frame_width: int = 640,
        frame_height: int = 480,
        focal_length_px: float = 500.0,
        deadband_px: float = 35.0,
        speed_m_s: float = 0.164,       # PWM 50 calibrated speed
        turn_rate_deg_s: float = 33.0,   # PWM 80 calibrated turn rate
        search_step_s: float = 1.10,    # Wide sector turn during search (~35 deg)
        max_step_s: float = 0.50,       # Step duration ceiling
        max_lost_frames: int = 5        # Allowed frames to settle before aborting
    ):
        self.target_label = target_label.lower()
        self.target_distance_m = float(target_distance_m)
        self.expected_width_m = float(expected_width_m) if expected_width_m is not None else None
        self.disambiguation = str(disambiguation).lower() if disambiguation else None
        self.stored_heading_deg = float(stored_heading_deg) if stored_heading_deg is not None else None
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.u_center = frame_width / 2.0
        self.focal_length = focal_length_px
        self.deadband_px = deadband_px
        self.speed_m_s = speed_m_s
        self.turn_rate_deg_s = turn_rate_deg_s
        self.search_step_s = search_step_s
        self.max_step_s = max_step_s
        self.max_lost_frames = max_lost_frames

        self.state = "idle"  # 'idle', 'memory_turn', 'searching', 'centering', 'approaching', 'arrived', 'lost'
        self.search_angle_accum_deg = 0.0
        self.consecutive_lost_frames = 0
        self.last_target_detection = None
        self.last_target_box = None
        self.memory_turn_executed = False
        self.filtered_dist_m = None

    def _compute_iou(self, box_a: List[float], box_b: List[float]) -> float:
        """Compute intersection-over-union between two bounding boxes [x1, y1, x2, y2]."""
        xa1, ya1, xa2, ya2 = box_a
        xb1, yb1, xb2, yb2 = box_b
        inter_x1 = max(xa1, xb1)
        inter_y1 = max(ya1, yb1)
        inter_x2 = min(xa2, xb2)
        inter_y2 = min(ya2, yb2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        area_a = max(1e-6, (xa2 - xa1) * (ya2 - ya1))
        area_b = max(1e-6, (xb2 - xb1) * (yb2 - yb1))
        union_area = area_a + area_b - inter_area
        return inter_area / union_area if union_area > 0 else 0.0

    def find_target(self, detections: List[Dict[str, Any]], min_conf: float = 0.25) -> Optional[Dict[str, Any]]:
        """Find the matching detection for target_label, handling disambiguation and persistence."""
        candidates = []
        for det in detections:
            label = det.get("label", "").lower()
            conf = det.get("confidence", 0.0)
            if (self.target_label in label or label in self.target_label) and conf >= min_conf:
                candidates.append(det)

        if not candidates:
            return None

        # 1. Target Persistence: If already locked onto an instance, maintain identity across frames
        if self.last_target_box is not None:
            def tracking_score(d):
                box = d["box"]
                iou = self._compute_iou(box, self.last_target_box)
                u_curr = (box[0] + box[2]) / 2.0
                u_prev = (self.last_target_box[0] + self.last_target_box[2]) / 2.0
                dist_norm = abs(u_curr - u_prev) / self.frame_width
                return 3.0 * iou + (1.0 - dist_norm)

            candidates.sort(key=tracking_score, reverse=True)
            chosen = candidates[0]
            self.last_target_box = chosen["box"]
            return chosen

        # 2. First acquisition: apply disambiguation selector if multiple candidates exist
        if len(candidates) > 1 and self.disambiguation:
            if self.disambiguation in ("left", "leftmost"):
                candidates.sort(key=lambda d: (d["box"][0] + d["box"][2]) / 2.0)
            elif self.disambiguation in ("right", "rightmost"):
                candidates.sort(key=lambda d: (d["box"][0] + d["box"][2]) / 2.0, reverse=True)
            elif self.disambiguation in ("center", "middle"):
                candidates.sort(key=lambda d: abs((d["box"][0] + d["box"][2]) / 2.0 - self.u_center))
            elif self.disambiguation in ("nearest", "largest", "closest"):
                candidates.sort(key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]), reverse=True)
            elif self.disambiguation in ("farthest", "smallest"):
                candidates.sort(key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))
            elif self.disambiguation.isdigit():
                candidates.sort(key=lambda d: (d["box"][0] + d["box"][2]) / 2.0)
                idx = min(max(0, int(self.disambiguation) - 1), len(candidates) - 1)
                chosen = candidates[idx]
                self.last_target_box = chosen["box"]
                return chosen

            chosen = candidates[0]
            self.last_target_box = chosen["box"]
            return chosen

        # Default fallback: largest confidence-weighted area
        def score(d):
            box = d.get("box", [0, 0, 0, 0])
            area = max(1.0, (box[2] - box[0]) * (box[3] - box[1]))
            return math.sqrt(area) * d.get("confidence", 0.0)

        candidates.sort(key=score, reverse=True)
        chosen = candidates[0]
        self.last_target_box = chosen["box"]
        return chosen

    def compute_step(self, detections: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Compute the next servoing action based on detected objects."""
        target = self.find_target(detections)

        if target is None:
            self.consecutive_lost_frames += 1
            if self.state == "approaching":
                if self.consecutive_lost_frames >= self.max_lost_frames:
                    self.state = "lost"
                    return {
                        "action": "stop",
                        "duration_s": 0.0,
                        "state": self.state,
                        "reason": "Target lost during approach"
                    }
                # Hold position briefly while camera settles
                return {"action": "stop", "duration_s": 0.0, "state": self.state}

            # Memory Turn: If target not immediately visible and stored heading is known, execute direct pivot
            if self.stored_heading_deg is not None and not self.memory_turn_executed:
                self.memory_turn_executed = True
                h = (self.stored_heading_deg + 180.0) % 360.0 - 180.0
                if abs(h) > 5.0:
                    self.state = "memory_turn"
                    action = "pwm_spin_right" if h > 0 else "pwm_spin_left"
                    duration = min(self.max_step_s, max(0.10, abs(h) / self.turn_rate_deg_s))
                    return {
                        "action": action,
                        "duration_s": round(duration, 3),
                        "state": self.state,
                        "heading_deg": round(self.stored_heading_deg, 1)
                    }

            # Search phase: spin in-place to locate target
            self.filtered_dist_m = None
            if self.search_angle_accum_deg >= 360.0:
                self.state = "lost"
                return {
                    "action": "stop",
                    "duration_s": 0.0,
                    "state": self.state,
                    "reason": "Target not found after full 360-degree scan"
                }

            self.state = "searching"
            step_duration = self.search_step_s
            step_angle = step_duration * self.turn_rate_deg_s
            self.search_angle_accum_deg += step_angle
            return {
                "action": "pwm_spin_right",
                "duration_s": step_duration,
                "state": self.state,
                "scanned_deg": round(self.search_angle_accum_deg, 1)
            }

        # Target found! Reset search tracking
        self.consecutive_lost_frames = 0
        self.search_angle_accum_deg = 0.0
        self.last_target_detection = target

        box = target["box"]
        u_obj = (box[0] + box[2]) / 2.0
        w_px = max(1.0, box[2] - box[0])

        spatial = target.get("spatial", {})
        raw_dist = spatial.get("distance_m")
        ground_xy = spatial.get("ground_xy_m")

        # Elevated objects (e.g. hanging towels, windows, wall art) do not rest on the floor.
        # Also, if ground projection produces X <= 0.20m, the contact pixel is above the floor horizon.
        elevated_objects = {"towel", "window", "clock", "picture", "curtain", "lamp", "tv"}
        is_elevated = (self.target_label in elevated_objects)
        is_ground_unphysical = (ground_xy is not None and ground_xy[0] <= 0.20)

        if is_elevated or is_ground_unphysical or raw_dist is None:
            # Pinhole optical distance from bounding box width
            if self.expected_width_m is not None and self.expected_width_m > 0:
                ref_width = self.expected_width_m
            else:
                ref_width = {
                    "chair": 0.55, "window": 1.0, "doorway": 0.85,
                    "table": 0.80, "couch": 1.40, "person": 0.45,
                    "towel": 0.28, "backpack": 0.40, "shoe": 0.25,
                    "shoes": 0.25, "laptop": 0.35, "cup": 0.10, "bottle": 0.08
                }.get(self.target_label, 0.50)
            distance_m = (self.focal_length * ref_width) / w_px
        else:
            distance_m = raw_dist

        # Temporal smoothing to prevent single-frame fold variations from causing distance jumps
        if self.filtered_dist_m is None:
            self.filtered_dist_m = distance_m
        else:
            self.filtered_dist_m = 0.65 * distance_m + 0.35 * self.filtered_dist_m
        effective_dist_m = self.filtered_dist_m

        error_u = u_obj - self.u_center

        # 1. Centering (yaw alignment) MUST take precedence
        # Uses pivot turns during approach so neither track reverses while driving forward
        if abs(error_u) > self.deadband_px:
            self.state = "centering"
            angle_deg = math.degrees(abs(math.atan2(error_u, self.focal_length)))
            action = "pwm_pivot_right" if error_u > 0 else "pwm_pivot_left"
            duration = min(self.max_step_s, max(0.08, angle_deg / self.turn_rate_deg_s))
            return {
                "action": action,
                "duration_s": round(duration, 3),
                "state": self.state,
                "distance_m": round(effective_dist_m, 3),
                "error_u_px": round(error_u, 1),
                "target_det": target
            }

        # 2. Check arrival threshold ONLY when target is well-centered
        if effective_dist_m <= self.target_distance_m:
            self.state = "arrived"
            return {
                "action": "stop",
                "duration_s": 0.0,
                "state": self.state,
                "distance_m": round(effective_dist_m, 3),
                "error_u_px": round(error_u, 1),
                "target_det": target
            }

        # 3. Well-centered and not arrived: advance forward
        self.state = "approaching"
        dist_to_go = max(0.0, effective_dist_m - self.target_distance_m)
        duration = min(self.max_step_s, max(0.10, dist_to_go / self.speed_m_s))
        return {
            "action": "pwm_forward",
            "duration_s": round(duration, 3),
            "state": self.state,
            "distance_m": round(effective_dist_m, 3),
            "error_u_px": round(error_u, 1),
            "target_det": target
        }
