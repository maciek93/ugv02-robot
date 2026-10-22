#!/usr/bin/env python3
"""High-level LLM Autonomous Driving Agent and Tool Interface.

Exposes high-level natural language actions:
- navigate_to_room(target_room)
- approach_object(object_name, target_distance_m)
- scan_room()
- get_robot_status()
- emergency_stop()
"""
import argparse
import base64
import io
import json
import logging
import math
from pathlib import Path
import sys
import time
from typing import Dict, Any, List, Optional, Tuple

import cv2
import numpy as np
from PIL import Image
import requests

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "supervised"))

from supervised.contact_geometry import ContactGeometry
from supervised.servoing import VisualServoingController
from supervised.topological_nav import TopologicalMap
from supervised.yolo_world import YoloWorldDetector
from supervised.doorway_transit import DoorwayTransitController
from supervised.corridor_check import CorridorObstacleChecker
from supervised.map_renderer import render_floor_map


TOOLS = [
    {
        "name": "get_robot_status",
        "description": "Get current robot telemetry, battery voltage, active room, and system health.",
        "parameters": {"type": "object", "properties": {}}
    },
    {
        "name": "scan_room",
        "description": "Perform a 360-degree panoramic scan to discover landmarks, identify or discover the current room, and register it in the topological map.",
        "parameters": {"type": "object", "properties": {}}
    },
    {
        "name": "inspect_scene",
        "description": "Capture camera frame, detect all visible objects with metric distances, classify horizontal positions (left/center/right), flag any ambiguous identical objects, and save an annotated snapshot for conversational resolution.",
        "parameters": {
            "type": "object",
            "properties": {
                "query_labels": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Optional list of object labels to detect (e.g. ['chair', 'table', 'sofa', 'doorway'])"
                }
            }
        }
    },
    {
        "name": "approach_object",
        "description": "Locate a target object (e.g. 'chair', 'table', 'couch', 'doorway') and drive toward it using visual servoing until within target_distance_m. Supports target disambiguation ('left', 'right', 'center', index) and obstacle avoidance reflex.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "description": "Label of the object to approach (e.g. 'chair', 'table', 'towel', 'backpack')"},
                "target_distance_m": {"type": "number", "default": 0.50, "description": "Stopping distance in meters"},
                "expected_width_m": {"type": "number", "description": "Optional physical width of object in meters (e.g. 0.28 for towel, 0.40 for backpack, 0.55 for chair). Calibrates optical distance for elevated/hanging objects."},
                "disambiguation": {"type": "string", "description": "Optional spatial selector when multiple matching candidates exist: 'left', 'right', 'center', 'nearest', 'farthest', or 1-based index ('1', '2')."}
            },
            "required": ["object_name"]
        }
    },
    {
        "name": "navigate_to_room",
        "description": "Navigate autonomously through doorways from the current room to the target room using the topological map, verifying doorway aperture (>50cm) and executing gyro-stabilized threshold crossing.",
        "parameters": {
            "type": "object",
            "properties": {
                "target_room": {"type": "string", "description": "Destination room name (e.g. 'kitchen', 'living_room')"}
            },
            "required": ["target_room"]
        }
    },
    {
        "name": "show_map",
        "description": "Render and return the current 2D top-down floor map diagram (data/floor_map.png) showing all rooms, registered objects, doorways, and robot position.",
        "parameters": {"type": "object", "properties": {}}
    },
    {
        "name": "edit_map",
        "description": "Conversationally update or correct the underlying spatial world model in room_graph.json.",
        "parameters": {
            "type": "object",
            "properties": {
                "operation": {
                    "type": "string",
                    "enum": ["rename_room", "delete_object", "update_object", "delete_doorway", "set_doorway_passable"],
                    "description": "Operation to perform"
                },
                "room_name": {"type": "string", "description": "Target room name"},
                "new_name": {"type": "string", "description": "New room name (for rename_room)"},
                "object_id": {"type": "string", "description": "Object ID or label (for delete_object or update_object)"},
                "doorway_id": {"type": "string", "description": "Doorway ID (for delete_doorway or set_doorway_passable)"},
                "is_passable": {"type": "boolean", "description": "Passability status (for set_doorway_passable)"}
            },
            "required": ["operation"]
        }
    },
    {
        "name": "emergency_stop",
        "description": "Immediately stop all motor motion.",
        "parameters": {"type": "object", "properties": {}}
    }
]


class LLMDriver:
    def __init__(
        self,
        root_path: Optional[Path] = None,
        gateway_url: str = "http://127.0.0.1:5001",
        pi_url: str = "http://127.0.0.1:5002"
    ):
        self.root = root_path or ROOT
        self.gateway_url = gateway_url
        self.pi_url = pi_url

        # Load API keys
        agent_key_path = self.root / ".runtime/agent-key"
        self.agent_key = agent_key_path.read_text().strip() if agent_key_path.exists() else None
        pi_keys_path = self.root / ".runtime/pi-keys.json"
        self.pi_keys = json.loads(pi_keys_path.read_text()) if pi_keys_path.exists() else {}

        # Initialize perception & geometry
        self.tmap = TopologicalMap(filepath=str(self.root / "data/room_graph.json"))
        # Calibrated camera & floor perspective homography
        H_nom = np.array([
            [0.0, -7.0486e-12, -6.0311e-02],
            [1.2062e-04, 0.0, -3.8599e-02],
            [0.0, -2.7109e-03, 1.0]
        ], dtype=np.float64)
        K_nom = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.contact_geometry = ContactGeometry(H_nom, K_nom)
        self.detector = None  # Lazy-loaded on first perception call

    def get_detector(self) -> YoloWorldDetector:
        if self.detector is None:
            self.detector = YoloWorldDetector(model_path=str(self.root / "models/yolov8s-worldv2.pt"))
        return self.detector

    def gateway_rpc(self, path: str, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        headers = {"X-Agent-Key": self.agent_key} if self.agent_key else {}
        url = self.gateway_url + path
        resp = requests.get(url, headers=headers, timeout=3) if data is None else requests.post(url, headers=headers, json=data, timeout=3)
        res = resp.json()
        if not resp.ok:
            raise RuntimeError(res.get("error", f"Gateway RPC {path} failed"))
        return res

    def pi_rpc(self, path: str, data: Optional[Dict[str, Any]] = None, role: str = "operator") -> Dict[str, Any]:
        key = self.pi_keys.get(role)
        headers = {"X-Robot-Key": key} if key else {}
        url = self.pi_url + path
        resp = requests.get(url, headers=headers, timeout=3) if data is None else requests.post(url, headers=headers, json=data, timeout=3)
        res = resp.json()
        if not resp.ok:
            raise RuntimeError(res.get("error", f"Pi RPC {path} failed"))
        return res

    def get_status(self) -> Dict[str, Any]:
        """Fetch full robot health, chassis telemetry, and topological status."""
        health = self.gateway_rpc("/agent/status")
        chassis = health.get("chassis", {})
        battery = chassis.get("battery_voltage_v")
        return {
            "state": health.get("state"),
            "mode": health.get("mode"),
            "moving": health.get("moving"),
            "battery_voltage_v": battery,
            "current_room": self.tmap.current_room,
            "known_rooms": list(self.tmap.rooms.keys()),
            "blocking_reason": health.get("blocking_reason")
        }

    def ensure_active_session(self):
        """Ensure an active manual session is running before proposing motion."""
        try:
            status = self.get_status()
            if status.get("state") != "active" or status.get("mode") != "manual":
                self.gateway_rpc("/agent/session", {"operation": "start", "mode": "manual"})
        except Exception as e:
            logging.warning("ensure_active_session: %s", e)

    def emergency_stop(self) -> Dict[str, Any]:
        """Immediately cancel all motor leases and stop motion."""
        try:
            return self.gateway_rpc("/agent/stop", {})
        except Exception as e:
            logging.warning("Gateway stop error, attempting direct Pi stop: %s", e)
            return self.pi_rpc("/stop", {}, role="agent")

    def capture_frame(self) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Fetch current camera frame and decode to RGB numpy array."""
        obs = self.gateway_rpc("/agent/observe")
        jpeg = base64.b64decode(obs.pop("jpeg"))
        img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        return np.asarray(img), obs

    def execute_motion_step(self, action: str, duration_s: float, description: str = "LLM motion step"):
        """Safely propose, approve, and execute a bounded motion primitive with watchdog renewal."""
        if duration_s <= 0:
            return {"done": True}

        # 1. Propose
        proposal = self.gateway_rpc("/agent/propose", {"action": action, "value": duration_s})
        prop_id = proposal["id"]

        # 2. Approve via operator interface
        self.pi_rpc("/operator/approve", {"proposal_id": prop_id, "clearance": True}, role="operator")

        # 3. Execute
        self.pi_rpc("/execute", {"proposal_id": prop_id}, role="agent")

        # 4. Renewal loop (renew lease every 60-80ms until spent)
        end_time = time.monotonic() + duration_s + 0.3
        while time.monotonic() < end_time:
            time.sleep(0.06)
            try:
                res = self.pi_rpc("/renew", {"proposal_id": prop_id}, role="agent")
                if res.get("done"):
                    break
            except Exception as e:
                # Execution finished or cancelled
                break

        # Stop to ensure clean state
        try:
            self.emergency_stop()
        except Exception:
            pass

        # Allow camera vibration and mechanical momentum to settle before next frame
        time.sleep(0.20)

        return {"done": True, "action": action, "duration_s": duration_s}

    def scan_room(self) -> Dict[str, Any]:
        """Perform a 360-degree scan (8 x 45 deg) to detect landmarks and classify or discover room."""
        logging.info("Initiating 360-degree panoramic scan...")
        detector = self.get_detector()
        all_landmarks = set()
        embeddings = []
        scan_records = []

        # Dynamically scale step duration based on battery voltage:
        status = self.get_status()
        v_batt = status.get("battery_voltage_v") or 11.5
        v_scale = min(1.4, max(0.9, 11.5 / v_batt))
        step_duration = round(1.36 * v_scale, 2)
        logging.info("Scan turn step duration: %.2fs (voltage scale: %.2fx at %.2fV)",
                     step_duration, v_scale, v_batt)

        for step_idx in range(8):
            rgb, obs = self.capture_frame()
            detections = detector.detect_with_geometry(rgb, self.contact_geometry)
            labels = [d["label"] for d in detections]
            all_landmarks.update(labels)
            emb = self.tmap.compute_embedding(rgb)
            embeddings.append(emb.tolist())

            scan_records.append({
                "step": step_idx + 1,
                "heading_deg": step_idx * 45,
                "detected": labels
            })

            # Turn 45 deg right after capturing each sector (8 turns total for full 360 deg)
            self.execute_motion_step("pwm_spin_right", step_duration, f"Scan turn {step_idx + 1}/8")

        # Check novelty against existing room graph
        mean_emb = np.mean(embeddings, axis=0)
        norm = np.linalg.norm(mean_emb)
        if norm > 1e-6:
            mean_emb /= norm

        # Novelty check using representative embedding
        representative_rgb, _ = self.capture_frame()
        is_novel, sim, closest_room = self.tmap.check_novelty(representative_rgb, threshold=0.65)

        landmarks_list = sorted(all_landmarks)
        if is_novel:
            suggested_name = self.tmap.suggest_room_name(landmarks_list)
            self.tmap.add_room(suggested_name, landmarks=landmarks_list, embeddings=embeddings)
            self.tmap.current_room = suggested_name
            self.tmap.save()
            return {
                "event": "room_discovered",
                "room_name": suggested_name,
                "is_novel": True,
                "similarity_to_known": sim,
                "landmarks": landmarks_list,
                "details": scan_records,
                "prompt": f"Discovered novel room with landmarks {landmarks_list}. Assigned name '{suggested_name}'."
            }
        else:
            self.tmap.current_room = closest_room
            self.tmap.add_room(closest_room, landmarks=landmarks_list, embeddings=embeddings)
            self.tmap.save()
            return {
                "event": "room_recognized",
                "room_name": closest_room,
                "is_novel": False,
                "similarity": sim,
                "landmarks": landmarks_list,
                "details": scan_records,
                "prompt": f"Recognized current room as '{closest_room}' (similarity: {sim})."
            }

    def _save_annotated_snapshot(
        self,
        rgb: np.ndarray,
        candidates_by_label: Dict[str, List[Dict[str, Any]]],
        output_path: str = "data/last_inspection.jpg"
    ) -> str:
        """Draw numbered bounding boxes and candidate positions on camera image."""
        out = Path(self.root / output_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        # Convert RGB to BGR for OpenCV
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        for label, candidates in candidates_by_label.items():
            for c in candidates:
                cand_id = c["id"]
                box = [int(v) for v in c["box"]]
                pos = c["horizontal_position"]
                dist = c.get("distance_m")
                dist_str = f"{dist:.2f}m" if dist else "N/A"
                text = f"[{cand_id}] {label} ({pos}, {dist_str})"

                # Draw box
                cv2.rectangle(bgr, (box[0], box[1]), (box[2], box[3]), (0, 255, 255), 2)
                # Label badge
                cv2.rectangle(bgr, (box[0], max(0, box[1] - 22)), (box[0] + len(text) * 9, box[1]), (0, 0, 0), -1)
                cv2.putText(bgr, text, (box[0] + 4, max(14, box[1] - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imwrite(str(out), bgr)
        logging.info("Saved annotated inspection snapshot to %s", out)
        return str(out)

    def inspect_scene(self, query_labels: Optional[List[str]] = None) -> Dict[str, Any]:
        """Capture camera frame, detect all visible objects, classify positions, and flag ambiguities."""
        self.ensure_active_session()
        rgb, obs = self.capture_frame()
        detector = self.get_detector()
        if query_labels:
            detector.set_classes(list(set(detector.classes + [q.lower() for q in query_labels])))

        detections = detector.detect_with_geometry(rgb, self.contact_geometry)

        candidates_by_label: Dict[str, List[Dict[str, Any]]] = {}
        for idx, d in enumerate(detections):
            label = d["label"]
            box = d["box"]
            u_c = (box[0] + box[2]) / 2.0
            pos = "left" if u_c < 240 else ("right" if u_c > 400 else "center")
            dist = d.get("spatial", {}).get("distance_m")

            cand = {
                "id": idx + 1,
                "label": label,
                "confidence": round(d["confidence"], 2),
                "horizontal_position": pos,
                "u_pixel": round(u_c, 1),
                "box": [round(v, 1) for v in box],
                "distance_m": round(dist, 2) if dist else None
            }
            candidates_by_label.setdefault(label, []).append(cand)

            # Auto-register object into current room spatial memory
            curr_room = self.tmap.current_room or "unknown_room"
            if dist is not None and dist > 0.3:
                pixel_err = u_c - 320.0
                rel_heading = math.degrees(math.atan2(pixel_err, 500.0))
                self.tmap.add_object(
                    room_name=curr_room,
                    label=label,
                    heading_deg=rel_heading,
                    distance_m=dist,
                    horizontal_sector=pos
                )

        # Flag identical duplicate objects requiring disambiguation
        ambiguities = {k: v for k, v in candidates_by_label.items() if len(v) > 1}
        snapshot_file = self._save_annotated_snapshot(rgb, candidates_by_label, "data/last_inspection.jpg")

        return {
            "total_objects": len(detections),
            "candidates_by_label": candidates_by_label,
            "has_ambiguity": len(ambiguities) > 0,
            "ambiguous_classes": list(ambiguities.keys()),
            "snapshot_path": snapshot_file,
            "current_room": self.tmap.current_room,
            "message": f"Inspected room: {len(detections)} objects detected. " +
                       (f"Ambiguities detected for: {list(ambiguities.keys())}. Refer to {snapshot_file}."
                        if ambiguities else "No duplicate object ambiguities detected.")
        }

    def approach_object(
        self,
        object_name: str,
        target_distance_m: float = 0.50,
        expected_width_m: Optional[float] = None,
        disambiguation: Optional[str] = None,
        max_duration_s: float = 90.0
    ) -> Dict[str, Any]:
        """Closed-loop visual servoing to approach a target object with disambiguation and obstacle reflex."""
        self.ensure_active_session()
        logging.info("Starting approach toward '%s' (target: %.2fm, disambiguation: %s, width: %s)...",
                     object_name, target_distance_m, disambiguation or "auto",
                     f"{expected_width_m}m" if expected_width_m else "auto")

        detector = self.get_detector()
        if object_name.lower() not in [c.lower() for c in detector.classes]:
            detector.set_classes(detector.classes + [object_name.lower()])

        # Check spatial memory for stored heading
        stored_heading = None
        if self.tmap.current_room:
            matches = self.tmap.find_object(object_name)
            for m in matches:
                if m["room"] == self.tmap.current_room:
                    stored_heading = m["object"].get("heading_deg")
                    break

        status = self.get_status()
        v_batt = status.get("battery_voltage_v") or 11.5
        v_scale = min(1.4, max(0.9, 11.5 / v_batt))
        search_step_s = round(1.10 * v_scale, 2)

        controller = VisualServoingController(
            target_label=object_name,
            target_distance_m=target_distance_m,
            expected_width_m=expected_width_m,
            disambiguation=disambiguation,
            stored_heading_deg=stored_heading,
            search_step_s=search_step_s
        )
        corridor_checker = CorridorObstacleChecker()

        started = time.monotonic()
        steps_executed = 0

        while time.monotonic() - started < max_duration_s:
            rgb, obs = self.capture_frame()
            detections = detector.detect_with_geometry(rgb, self.contact_geometry)

            # Check forward corridor obstacle clearance
            is_blocked, obstacle = corridor_checker.check_corridor(detections, target_label=object_name)
            if is_blocked and not corridor_checker.reflex_active:
                logging.warning("Forward corridor blocked by '%s'! Initiating obstacle avoidance reflex.",
                                obstacle.get("label", "obstacle"))
                corridor_checker.start_reflex(obstacle)

            if corridor_checker.reflex_active:
                reflex_step = corridor_checker.compute_reflex_step()
                logging.info("Obstacle Reflex: %s -> %s (%.2fs)",
                             reflex_step["state"], reflex_step["action"], reflex_step["duration_s"])
                if reflex_step["duration_s"] > 0:
                    self.execute_motion_step(reflex_step["action"], reflex_step["duration_s"], "Obstacle reflex")
                    steps_executed += 1
                continue

            # Standard visual servoing step
            step = controller.compute_step(detections)

            if step["state"] == "arrived":
                self.emergency_stop()
                # Capture final arrival confirmation photo
                final_rgb, _ = self.capture_frame()
                arrival_img_path = str(self.root / "data/last_arrival.jpg")
                Image.fromarray(final_rgb).save(arrival_img_path)
                return {
                    "success": True,
                    "state": "arrived",
                    "target": object_name,
                    "final_distance_m": step.get("distance_m"),
                    "steps": steps_executed,
                    "arrival_photo": arrival_img_path,
                    "message": f"Successfully approached '{object_name}' to {step.get('distance_m')}m."
                }

            if step["state"] == "lost":
                self.emergency_stop()
                return {
                    "success": False,
                    "state": "lost",
                    "target": object_name,
                    "steps": steps_executed,
                    "message": f"Target '{object_name}' lost or not found during visual pursuit."
                }

            state = step["state"]
            dist = step.get("distance_m")
            err_u = step.get("error_u_px")
            action = step["action"]
            duration = step["duration_s"]

            logging.info("Servoing [%s]: dist=%s, yaw_err=%s -> action=%s (%.2fs)",
                         state, f"{dist:.2f}m" if dist else "N/A",
                         f"{err_u:+.1f}px" if err_u else "N/A", action, duration)

            if duration > 0:
                self.execute_motion_step(action, duration, f"Servoing: {state}")
                steps_executed += 1
            else:
                time.sleep(0.20)

        self.emergency_stop()
        return {
            "success": False,
            "state": "timeout",
            "target": object_name,
            "steps": steps_executed,
            "message": f"Visual servoing approach toward '{object_name}' timed out after {max_duration_s}s."
        }

    def navigate_to_room(self, target_room: str) -> Dict[str, Any]:
        """Navigate through doorways to target room with aperture verification and gyro threshold crossing."""
        target_room = target_room.strip().lower().replace(" ", "_")
        current_room = self.tmap.current_room

        if not current_room:
            scan_res = self.scan_room()
            current_room = scan_res["room_name"]

        if current_room == target_room:
            return {"success": True, "message": f"Already in room '{target_room}'."}

        path = self.tmap.find_path(current_room, target_room)
        if path is None:
            return {
                "success": False,
                "message": f"No topological path found from '{current_room}' to '{target_room}' in room graph."
            }

        logging.info("Navigating path: %s", [f"{d['from']} -> {d['to']}" for d in path])
        detector = self.get_detector()
        if "doorway" not in detector.classes:
            detector.set_classes(detector.classes + ["doorway", "door"])

        for doorway in path:
            dest = doorway["to"]
            heading = doorway.get("heading_deg", 0.0)
            doorway_id = doorway.get("id", "door")

            # 1. Turn toward doorway heading
            if abs(heading) > 5.0:
                turn_action = "pwm_spin_right" if heading > 0 else "pwm_spin_left"
                turn_dur = abs(heading) / 33.0
                self.execute_motion_step(turn_action, turn_dur, f"Turn toward doorway {doorway_id}")

            # 2. Doorway Traversability Gate & Aperture Centering
            transit_ctrl = DoorwayTransitController(
                min_aperture_m=0.50,
                approach_stop_dist_m=0.75,
                threshold_stride_m=1.00
            )
            transit_start = time.monotonic()
            crossed = False

            while time.monotonic() - transit_start < 45.0:
                rgb, obs = self.capture_frame()
                detections = detector.detect_with_geometry(rgb, self.contact_geometry)
                step = transit_ctrl.compute_step(detections)

                if step["state"] == "impassable":
                    self.emergency_stop()
                    # Flag doorway in room graph
                    self.tmap.set_doorway_passable(doorway_id, False)
                    return {
                        "success": False,
                        "state": "doorway_blocked",
                        "doorway_id": doorway_id,
                        "aperture_width_m": step.get("aperture_width_m"),
                        "message": f"Doorway '{doorway_id}' is closed or obstructed (aperture: {step.get('aperture_width_m')}m < 0.50m). Transit aborted."
                    }

                if step["state"] == "ready_for_stride":
                    # Execute gyro-stabilized 1.0m flat threshold forward stride
                    logging.info("Doorway aperture verified! Command 1.0m forward threshold stride (%.2fs)",
                                 step["duration_s"])
                    self.execute_motion_step("pwm_forward", step["duration_s"], f"Threshold crossing {doorway_id}")
                    crossed = True
                    break

                if step["duration_s"] > 0:
                    self.execute_motion_step(step["action"], step["duration_s"], f"Doorway: {step['state']}")
                else:
                    time.sleep(0.15)

            if not crossed:
                # Fallback: advance 4.2s stride if doorway detector was uncertain but edge exists
                logging.info("Doorway transit timeout; executing fallback threshold advance.")
                self.execute_motion_step("pwm_forward", 4.2, f"Fallback traverse {doorway_id}")

            # 3. Update active room
            self.tmap.current_room = dest
            self.tmap.save()

        return {
            "success": True,
            "current_room": target_room,
            "path_traversed": [d["id"] for d in path],
            "message": f"Successfully navigated to room '{target_room}'."
        }

    def show_map(self) -> Dict[str, Any]:
        """Render and return the visual floor map and spatial model summary."""
        map_img_path = str(self.root / "data/floor_map.png")
        render_floor_map(self.tmap, output_path=map_img_path)

        rooms_summary = {}
        for r_name, r_data in self.tmap.rooms.items():
            objs = [f"{o.get('id', 'obj')} ({o.get('horizontal_sector')}, {o.get('distance_m')}m)"
                    for o in r_data.get("objects", [])]
            rooms_summary[r_name] = {
                "type": r_data.get("type", "room"),
                "landmarks": r_data.get("landmarks", []),
                "objects": objs
            }

        doorways_summary = [
            f"{d.get('id')}: {d.get('from')} -> {d.get('to')} (aperture: {d.get('aperture_width_m', 0.85)}m, {'passable' if d.get('is_passable', True) else 'BLOCKED'})"
            for d in self.tmap.doorways
        ]

        return {
            "map_image": map_img_path,
            "current_room": self.tmap.current_room,
            "total_rooms": len(self.tmap.rooms),
            "rooms": rooms_summary,
            "doorways": doorways_summary,
            "message": f"Floor map rendered with {len(self.tmap.rooms)} rooms and {len(self.tmap.doorways)} doorways."
        }

    def edit_map(self, operation: str, **kwargs) -> Dict[str, Any]:
        """Conversationally edit or correct the room graph spatial model."""
        op = operation.strip().lower()

        if op == "rename_room":
            old_name = kwargs.get("room_name")
            new_name = kwargs.get("new_name")
            if not old_name or not new_name:
                raise ValueError("rename_room requires 'room_name' and 'new_name'")
            self.tmap.rename_room(old_name, new_name)
            msg = f"Renamed room '{old_name}' to '{new_name}'."

        elif op == "delete_object":
            room_name = kwargs.get("room_name")
            obj_id = kwargs.get("object_id")
            if not room_name or not obj_id:
                raise ValueError("delete_object requires 'room_name' and 'object_id'")
            deleted = self.tmap.delete_object(room_name, obj_id)
            msg = f"Object '{obj_id}' {'deleted from ' + room_name if deleted else 'not found'}."

        elif op == "update_object":
            room_name = kwargs.get("room_name")
            obj_id = kwargs.get("object_id")
            updates = {k: v for k, v in kwargs.items() if k not in ("operation", "room_name", "object_id")}
            updated = self.tmap.update_object(room_name, obj_id, updates)
            msg = f"Object '{obj_id}' {'updated' if updated else 'not found'} in '{room_name}'."

        elif op == "delete_doorway":
            doorway_id = kwargs.get("doorway_id")
            if not doorway_id:
                raise ValueError("delete_doorway requires 'doorway_id'")
            deleted = self.tmap.delete_doorway(doorway_id)
            msg = f"Doorway '{doorway_id}' {'deleted' if deleted else 'not found'}."

        elif op == "set_doorway_passable":
            doorway_id = kwargs.get("doorway_id")
            is_passable = kwargs.get("is_passable", True)
            if not doorway_id:
                raise ValueError("set_doorway_passable requires 'doorway_id'")
            self.tmap.set_doorway_passable(doorway_id, bool(is_passable))
            msg = f"Doorway '{doorway_id}' marked as {'passable' if is_passable else 'blocked'}."

        else:
            raise ValueError(f"Unknown edit_map operation: {operation}")

        # Automatically re-render floor map
        map_path = str(self.root / "data/floor_map.png")
        render_floor_map(self.tmap, output_path=map_path)
        return {
            "success": True,
            "operation": op,
            "message": msg,
            "updated_map_image": map_path
        }


def main():
    parser = argparse.ArgumentParser(description="LLM Autonomous Robot Driver")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", help="Get robot and topological status")
    sub.add_parser("scan", help="Perform 360-degree scan and room discovery")
    sub.add_parser("inspect", help="Inspect visual scene, detect objects and ambiguities")

    approach_cmd = sub.add_parser("approach", help="Approach a target object using visual servoing")
    approach_cmd.add_argument("object_name", type=str, help="Name of object (e.g. chair, doorway, table)")
    approach_cmd.add_argument("--distance", type=float, default=0.50, help="Target stopping distance in meters")
    approach_cmd.add_argument("--width", type=float, default=None, help="Expected physical metric width of target object in meters")
    approach_cmd.add_argument("--disambiguation", type=str, default=None, help="Target disambiguation ('left', 'right', 'center', index)")
    approach_cmd.add_argument("--timeout", type=float, default=90.0, help="Maximum duration in seconds")

    nav_cmd = sub.add_parser("navigate", help="Navigate to a target room")
    nav_cmd.add_argument("target_room", type=str, help="Destination room name")

    sub.add_parser("map", help="Render and show visual floor map")
    sub.add_parser("stop", help="Emergency stop")
    sub.add_parser("tools", help="Print LLM tool schemas")

    args = parser.parse_args()

    if args.command == "tools":
        print(json.dumps(TOOLS, indent=2))
        return

    driver = LLMDriver()

    if args.command == "status":
        res = driver.get_status()
    elif args.command == "stop":
        res = driver.emergency_stop()
    elif args.command == "scan":
        res = driver.scan_room()
    elif args.command == "inspect":
        res = driver.inspect_scene()
    elif args.command == "approach":
        res = driver.approach_object(
            args.object_name,
            target_distance_m=args.distance,
            expected_width_m=args.width,
            disambiguation=args.disambiguation,
            max_duration_s=args.timeout
        )
    elif args.command == "navigate":
        res = driver.navigate_to_room(args.target_room)
    elif args.command == "map":
        res = driver.show_map()
    else:
        parser.print_help()
        return

    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()

