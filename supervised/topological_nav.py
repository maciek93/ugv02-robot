"""Topological room graph, scene novelty embedding, and automated room discovery."""
from collections import deque
import json
import logging
import math
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
import cv2
import numpy as np


ROOM_LANDMARK_MAP = {
    "kitchen": ["refrigerator", "fridge", "microwave", "oven", "sink", "toaster", "kettle"],
    "living_room": ["sofa", "couch", "armchair", "tv", "television", "coffee table"],
    "bedroom": ["bed", "wardrobe", "nightstand", "pillow"],
    "office": ["desk", "computer", "laptop", "monitor", "office chair", "bookshelf", "bookcase"],
    "dining_room": ["dining table", "dining chair"],
    "bathroom": ["toilet", "sink", "bathtub", "shower"],
    "hallway": ["doorway", "door", "radiator", "coat rack"]
}


class SceneFeatureExtractor:
    """Computes a normalized 512-d multi-scale color and gradient structural descriptor."""

    def extract(self, rgb: np.ndarray) -> np.ndarray:
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("RGB image array with shape (H, W, 3) required")

        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        # Sobel edge orientation and magnitude
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        mag, ang = cv2.cartToPolar(gx, gy, angleInDegrees=True)
        ang = ang % 180.0

        h, w = rgb.shape[:2]
        gh, gw = h // 4, w // 4
        features = []

        for r in range(4):
            for c in range(4):
                cell_hsv = hsv[r * gh:(r + 1) * gh, c * gw:(c + 1) * gw]
                cell_mag = mag[r * gh:(r + 1) * gh, c * gw:(c + 1) * gw]
                cell_ang = ang[r * gh:(r + 1) * gh, c * gw:(c + 1) * gw]

                hist_h = cv2.calcHist([cell_hsv], [0], None, [8], [0, 180]).flatten()
                hist_s = cv2.calcHist([cell_hsv], [1], None, [8], [0, 256]).flatten()
                hist_v = cv2.calcHist([cell_hsv], [2], None, [8], [0, 256]).flatten()
                hist_grad, _ = np.histogram(cell_ang, bins=8, range=(0, 180), weights=cell_mag)

                cell_feat = np.concatenate([hist_h, hist_s, hist_v, hist_grad])
                norm = np.linalg.norm(cell_feat)
                if norm > 1e-6:
                    cell_feat /= norm
                features.append(cell_feat)

        vec = np.concatenate(features).astype(np.float32)
        norm = np.linalg.norm(vec)
        if norm > 1e-6:
            vec /= norm
        return vec


class TopologicalMap:
    """Manages rooms, doorways, topological navigation and automated room discovery."""

    def __init__(self, filepath: Optional[str] = "data/room_graph.json"):
        self.filepath = Path(filepath) if filepath else None
        self.extractor = SceneFeatureExtractor()
        self.rooms: Dict[str, Dict[str, Any]] = {}
        self.doorways: List[Dict[str, Any]] = []
        self.current_room: Optional[str] = None
        self.load()

    def load(self):
        if self.filepath and self.filepath.exists():
            try:
                data = json.loads(self.filepath.read_text())
                self.rooms = data.get("rooms", {})
                self.doorways = data.get("doorways", [])
                self.current_room = data.get("current_room")
                # Ensure schema compatibility for version 2
                for r in self.rooms.values():
                    r.setdefault("objects", [])
                    r.setdefault("center_relative_m", [0.0, 0.0])
                for d in self.doorways:
                    if "aperture_width_m" not in d and "width_m" in d:
                        d["aperture_width_m"] = d["width_m"]
                    d.setdefault("is_passable", True)
            except Exception as e:
                logging.warning("Could not load topological map from %s: %s", self.filepath, e)
                self.rooms = {}
                self.doorways = []

    def save(self):
        if not self.filepath:
            return
        self.filepath.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "version": 2,
            "current_room": self.current_room,
            "rooms": self.rooms,
            "doorways": self.doorways
        }
        # Atomic write
        tmp = self.filepath.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2))
        tmp.replace(self.filepath)

    def add_room(
        self,
        name: str,
        landmarks: Optional[List[str]] = None,
        embeddings: Optional[List[List[float]]] = None,
        room_type: Optional[str] = None,
        center_relative_m: Optional[List[float]] = None
    ) -> str:
        name = name.strip().lower().replace(" ", "_")
        if name not in self.rooms:
            self.rooms[name] = {
                "name": name,
                "landmarks": sorted(list(set(landmarks or []))),
                "embeddings": embeddings or [],
                "type": room_type or ("corridor" if "hallway" in name or "corridor" in name else "room"),
                "center_relative_m": center_relative_m or [0.0, 0.0],
                "objects": []
            }
        else:
            if landmarks:
                existing = set(self.rooms[name].get("landmarks", []))
                self.rooms[name]["landmarks"] = sorted(existing | set(landmarks))
            if embeddings:
                self.rooms[name].setdefault("embeddings", []).extend(embeddings)
            if center_relative_m:
                self.rooms[name]["center_relative_m"] = center_relative_m
        self.save()
        return name

    def add_object(
        self,
        room_name: str,
        label: str,
        heading_deg: float,
        distance_m: float,
        expected_width_m: Optional[float] = None,
        horizontal_sector: Optional[str] = None,
        obj_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """Register a detected landmark object with polar and metric 2D coordinates."""
        room_name = room_name.strip().lower().replace(" ", "_")
        if room_name not in self.rooms:
            self.add_room(room_name)

        room = self.rooms[room_name]
        objects = room.setdefault("objects", [])

        clean_label = label.strip().lower()
        heading_deg = float(heading_deg % 360.0)
        distance_m = round(float(distance_m), 2)

        # Classify horizontal sector if not provided
        if not horizontal_sector:
            if heading_deg > 180:
                rel = heading_deg - 360.0
            else:
                rel = heading_deg
            if rel < -15:
                horizontal_sector = "left"
            elif rel > 15:
                horizontal_sector = "right"
            else:
                horizontal_sector = "center"

        # Compute metric coords relative to room center: x=east/right, y=north/forward
        rad = math.radians(heading_deg)
        room_center = room.get("center_relative_m", [0.0, 0.0])
        dx = round(distance_m * math.sin(rad), 2)
        dy = round(distance_m * math.cos(rad), 2)
        coords = [round(room_center[0] + dx, 2), round(room_center[1] + dy, 2)]

        # Check if an existing object with similar heading and label should be updated
        for existing in objects:
            if existing.get("label") == clean_label:
                h_diff = abs((existing.get("heading_deg", 0.0) - heading_deg + 180.0) % 360.0 - 180.0)
                if h_diff < 20.0 or (obj_id and existing.get("id") == obj_id):
                    existing.update({
                        "heading_deg": round(heading_deg, 1),
                        "distance_m": distance_m,
                        "coords_m": coords,
                        "horizontal_sector": horizontal_sector
                    })
                    if expected_width_m is not None:
                        existing["expected_width_m"] = round(expected_width_m, 2)
                    self.save()
                    return existing

        # New object
        if not obj_id:
            count = sum(1 for o in objects if clean_label in o.get("label", ""))
            obj_id = f"{clean_label}_{count + 1}"

        new_obj = {
            "id": obj_id,
            "label": clean_label,
            "horizontal_sector": horizontal_sector,
            "heading_deg": round(heading_deg, 1),
            "distance_m": distance_m,
            "coords_m": coords,
            "expected_width_m": round(expected_width_m, 2) if expected_width_m is not None else None
        }
        objects.append(new_obj)

        # Keep landmarks list updated
        landmarks = set(room.get("landmarks", []))
        landmarks.add(clean_label)
        room["landmarks"] = sorted(list(landmarks))

        self.save()
        return new_obj

    def delete_object(self, room_name: str, obj_id_or_label: str) -> bool:
        """Remove an object from a room by ID or label."""
        room_name = room_name.strip().lower().replace(" ", "_")
        if room_name not in self.rooms:
            return False

        room = self.rooms[room_name]
        objects = room.get("objects", [])
        initial_len = len(objects)

        target = obj_id_or_label.strip().lower()
        # First try exact ID match
        kept = [o for o in objects if o.get("id", "").lower() != target]
        if len(kept) == initial_len:
            # Fall back to label match
            kept = [o for o in objects if o.get("label", "").lower() != target]

        if len(kept) < initial_len:
            room["objects"] = kept
            # Refresh landmarks
            remaining_labels = sorted(list(set(o.get("label") for o in kept if o.get("label"))))
            room["landmarks"] = remaining_labels
            self.save()
            return True
        return False

    def update_object(self, room_name: str, obj_id: str, updates: Dict[str, Any]) -> bool:
        """Update properties of an existing object."""
        room_name = room_name.strip().lower().replace(" ", "_")
        if room_name not in self.rooms:
            return False

        room = self.rooms[room_name]
        for obj in room.get("objects", []):
            if obj.get("id", "").lower() == obj_id.strip().lower():
                obj.update(updates)
                if "heading_deg" in updates or "distance_m" in updates:
                    rad = math.radians(obj["heading_deg"])
                    dist = obj["distance_m"]
                    rc = room.get("center_relative_m", [0.0, 0.0])
                    obj["coords_m"] = [round(rc[0] + dist * math.sin(rad), 2), round(rc[1] + dist * math.cos(rad), 2)]
                self.save()
                return True
        return False

    def find_object(self, object_label: str) -> List[Dict[str, Any]]:
        """Find all registered objects across rooms matching label query."""
        query = object_label.strip().lower()
        matches = []
        for r_name, r_data in self.rooms.items():
            for obj in r_data.get("objects", []):
                if query in obj.get("label", "").lower() or query in obj.get("id", "").lower():
                    matches.append({
                        "room": r_name,
                        "object": obj
                    })
        return matches

    def get_room_objects(self, room_name: str) -> List[Dict[str, Any]]:
        """Return all objects registered in the specified room."""
        room_name = room_name.strip().lower().replace(" ", "_")
        return self.rooms.get(room_name, {}).get("objects", [])

    def set_room_center(self, room_name: str, x: float, y: float):
        """Set relative coordinates of room center and recompute all object coordinates."""
        room_name = room_name.strip().lower().replace(" ", "_")
        if room_name not in self.rooms:
            self.add_room(room_name)
        room = self.rooms[room_name]
        room["center_relative_m"] = [round(x, 2), round(y, 2)]
        for obj in room.get("objects", []):
            rad = math.radians(obj["heading_deg"])
            dist = obj["distance_m"]
            obj["coords_m"] = [round(x + dist * math.sin(rad), 2), round(y + dist * math.cos(rad), 2)]
        self.save()

    def rename_room(self, old_name: str, new_name: str):
        old_name = old_name.strip().lower().replace(" ", "_")
        new_name = new_name.strip().lower().replace(" ", "_")
        if old_name not in self.rooms:
            raise ValueError(f"Room '{old_name}' not found")
        if new_name in self.rooms and new_name != old_name:
            raise ValueError(f"Room '{new_name}' already exists")

        self.rooms[new_name] = self.rooms.pop(old_name)
        self.rooms[new_name]["name"] = new_name
        for d in self.doorways:
            if d.get("from") == old_name:
                d["from"] = new_name
            if d.get("to") == old_name:
                d["to"] = new_name
        if self.current_room == old_name:
            self.current_room = new_name
        self.save()

    def add_doorway(
        self,
        doorway_id: str,
        from_room: str,
        to_room: str,
        heading_deg: float = 0.0,
        width_m: float = 0.82,
        is_passable: bool = True
    ):
        from_room = from_room.strip().lower().replace(" ", "_")
        to_room = to_room.strip().lower().replace(" ", "_")
        if from_room not in self.rooms or to_room not in self.rooms:
            raise ValueError("Both rooms must exist before creating a doorway edge")

        # Check existing edge
        for d in self.doorways:
            if d.get("id") == doorway_id:
                d.update({
                    "from": from_room,
                    "to": to_room,
                    "heading_deg": round(heading_deg, 1),
                    "aperture_width_m": round(width_m, 2),
                    "is_passable": is_passable
                })
                self.save()
                return

        self.doorways.append({
            "id": doorway_id,
            "from": from_room,
            "to": to_room,
            "heading_deg": round(heading_deg, 1),
            "aperture_width_m": round(width_m, 2),
            "is_passable": is_passable
        })
        self.save()

    def delete_doorway(self, doorway_id: str) -> bool:
        """Remove a doorway edge."""
        initial_len = len(self.doorways)
        self.doorways = [d for d in self.doorways if d.get("id") != doorway_id]
        if len(self.doorways) < initial_len:
            self.save()
            return True
        return False

    def set_doorway_passable(self, doorway_id: str, is_passable: bool) -> bool:
        """Mark a doorway as passable or blocked/closed."""
        for d in self.doorways:
            if d.get("id") == doorway_id:
                d["is_passable"] = is_passable
                self.save()
                return True
        return False

    def find_path(self, start_room: str, goal_room: str) -> Optional[List[Dict[str, Any]]]:
        """Find the shortest sequence of doorways from start_room to goal_room."""
        start = start_room.strip().lower().replace(" ", "_")
        goal = goal_room.strip().lower().replace(" ", "_")
        if start not in self.rooms or goal not in self.rooms:
            return None
        if start == goal:
            return []

        # Build adjacency graph
        adj: Dict[str, List[Tuple[str, Dict[str, Any]]]] = {r: [] for r in self.rooms}
        for d in self.doorways:
            u, v = d["from"], d["to"]
            adj[u].append((v, d))
            # Bidirectional doorway support with flipped heading
            rev_d = {**d, "from": v, "to": u, "heading_deg": (d["heading_deg"] + 180.0) % 360.0}
            adj[v].append((u, rev_d))

        queue = deque([(start, [])])
        visited = {start}

        while queue:
            curr, path = queue.popleft()
            if curr == goal:
                return path
            for neighbor, edge in adj[curr]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [edge]))

        return None

    def compute_embedding(self, rgb_image: np.ndarray) -> np.ndarray:
        return self.extractor.extract(rgb_image)

    def check_novelty(
        self,
        rgb_image: np.ndarray,
        threshold: float = 0.65
    ) -> Tuple[bool, float, Optional[str]]:
        """Check if image represents an unmapped novel room.
        
        Returns:
            (is_novel, max_similarity, closest_room_name)
        """
        if not self.rooms:
            return True, 0.0, None

        current_emb = self.compute_embedding(rgb_image)
        max_sim = -1.0
        best_room = None

        for room_name, room_info in self.rooms.items():
            embeddings = room_info.get("embeddings", [])
            for emb_list in embeddings:
                emb = np.asarray(emb_list, dtype=np.float32)
                sim = float(np.dot(current_emb, emb))
                if sim > max_sim:
                    max_sim = sim
                    best_room = room_name

        is_novel = max_sim < threshold
        return is_novel, round(max_sim, 3), best_room

    def suggest_room_name(self, landmarks: List[str]) -> str:
        """Infer candidate room name based on detected landmark objects."""
        clean_landmarks = [l.strip().lower() for l in landmarks]
        scores = {}
        for room_type, keys in ROOM_LANDMARK_MAP.items():
            count = sum(1 for k in keys if any(k in l or l in k for l in clean_landmarks))
            if count > 0:
                scores[room_type] = count

        if scores:
            best_type = max(scores.items(), key=lambda item: item[1])[0]
            # Avoid naming collision
            candidate = best_type
            idx = 1
            while candidate in self.rooms:
                idx += 1
                candidate = f"{best_type}_{idx}"
            return candidate

        # Default unnamed
        idx = 1
        candidate = f"unnamed_room_{idx}"
        while candidate in self.rooms:
            idx += 1
            candidate = f"unnamed_room_{idx}"
        return candidate
