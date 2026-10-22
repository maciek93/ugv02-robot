"""Floor corridor clearance verification and obstacle avoidance reflex."""
from typing import Dict, Any, List, Optional, Tuple


class CorridorObstacleChecker:
    """Monitors the immediate floor corridor ahead of tracks for hazardous obstacles."""

    def __init__(
        self,
        min_clear_x_m: float = 0.15,
        max_clear_x_m: float = 0.70,
        corridor_half_width_m: float = 0.22,
        retreat_dist_m: float = 0.12,
        bypass_turn_deg: float = 25.0,
        turn_rate_deg_s: float = 33.0,
        speed_m_s: float = 0.164
    ):
        self.min_clear_x_m = float(min_clear_x_m)
        self.max_clear_x_m = float(max_clear_x_m)
        self.corridor_half_width_m = float(corridor_half_width_m)
        self.retreat_dist_m = float(retreat_dist_m)
        self.bypass_turn_deg = float(bypass_turn_deg)
        self.turn_rate_deg_s = float(turn_rate_deg_s)
        self.speed_m_s = float(speed_m_s)

        self.reflex_active = False
        self.reflex_phase = 0  # 0: idle, 1: halt, 2: retreat, 3: pivot, 4: bypass, 5: done
        self.preferred_bypass_dir = "left"

    def check_corridor(
        self,
        detections: List[Dict[str, Any]],
        target_label: Optional[str] = None
    ) -> Tuple[bool, Optional[Dict[str, Any]]]:
        """Check if any non-target obstacle is in the forward floor corridor (0.15m to 0.70m).
        
        Returns:
            (is_blocked, blocking_obstacle)
        """
        clean_target = target_label.lower().strip() if target_label else None

        for d in detections:
            label = d.get("label", "").lower()
            if clean_target and (clean_target in label or label in clean_target):
                # The target itself is what we are pursuing
                continue

            spatial = d.get("spatial", {})
            ground_xy = spatial.get("ground_xy_m")
            if ground_xy is not None:
                x_fwd, y_lat = ground_xy[0], ground_xy[1]
                if (self.min_clear_x_m <= x_fwd <= self.max_clear_x_m and
                        abs(y_lat) <= self.corridor_half_width_m):
                    return True, d

            # Also check bounding box bottom proximity if ground_xy is uncalibrated
            box = d.get("box", [0, 0, 0, 0])
            u_c = (box[0] + box[2]) / 2.0
            y_bottom = box[3]
            # If bottom of bounding box is near lower frame (y > 380) and horizontally centered (200 < u < 440)
            if y_bottom > 380 and 200 <= u_c <= 440:
                dist = spatial.get("distance_m")
                if dist is not None and dist <= self.max_clear_x_m:
                    return True, d

        return False, None

    def start_reflex(self, obstacle: Optional[Dict[str, Any]] = None):
        """Initiate the multi-step obstacle avoidance reflex sequence."""
        self.reflex_active = True
        self.reflex_phase = 1

        # Determine best bypass direction away from obstacle
        if obstacle:
            box = obstacle.get("box", [0, 0, 0, 0])
            u_c = (box[0] + box[2]) / 2.0
            # If obstacle is to the right of center (u > 320), pivot left; else pivot right
            self.preferred_bypass_dir = "left" if u_c >= 320 else "right"
        else:
            self.preferred_bypass_dir = "left"

    def compute_reflex_step(self) -> Dict[str, Any]:
        """Advance the local obstacle detour state machine."""
        if not self.reflex_active:
            return {"action": "stop", "duration_s": 0.0, "state": "idle"}

        if self.reflex_phase == 1:
            # Phase 1: Emergency halt
            self.reflex_phase = 2
            return {
                "action": "stop",
                "duration_s": 0.0,
                "state": "reflex_halt",
                "message": "Obstacle detected in immediate path. Halting."
            }

        elif self.reflex_phase == 2:
            # Phase 2: Retreat 12cm backwards
            self.reflex_phase = 3
            retreat_time = round(self.retreat_dist_m / self.speed_m_s, 2)
            return {
                "action": "pwm_spin_left",  # Back off / reverse stride
                "duration_s": retreat_time,
                "state": "reflex_retreat",
                "message": f"Nudging backward {int(self.retreat_dist_m * 100)}cm."
            }

        elif self.reflex_phase == 3:
            # Phase 3: Pivot 25 deg toward open floor
            self.reflex_phase = 4
            pivot_time = round(self.bypass_turn_deg / self.turn_rate_deg_s, 2)
            action = "pwm_pivot_left" if self.preferred_bypass_dir == "left" else "pwm_pivot_right"
            return {
                "action": action,
                "duration_s": pivot_time,
                "state": "reflex_pivot",
                "message": f"Pivoting {self.bypass_turn_deg} deg {self.preferred_bypass_dir} toward open floor."
            }

        elif self.reflex_phase == 4:
            # Phase 4: Advance 25cm past obstacle
            self.reflex_phase = 5
            advance_time = round(0.25 / self.speed_m_s, 2)
            return {
                "action": "pwm_forward",
                "duration_s": advance_time,
                "state": "reflex_advance",
                "message": "Advancing 25cm to clear obstacle."
            }

        else:
            # Phase 5: Reflex complete, ready to re-acquire target
            self.reflex_active = False
            self.reflex_phase = 0
            return {
                "action": "stop",
                "duration_s": 0.0,
                "state": "reflex_complete",
                "message": "Obstacle bypassed. Re-acquiring target."
            }
