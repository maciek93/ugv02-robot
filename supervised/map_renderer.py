"""Renderer for generating top-down visual floor maps from room_graph.json."""
import json
import logging
import math
from pathlib import Path
from typing import Optional, Dict, Any, List

import matplotlib
matplotlib.use("Agg")  # Headless backend
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D
import numpy as np

from supervised.topological_nav import TopologicalMap


def render_floor_map(
    tmap: TopologicalMap,
    output_path: str = "data/floor_map.png"
) -> str:
    """Render top-down 2D diagram of rooms, doorways, objects and robot position."""
    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10, 8), dpi=150)
    ax.set_facecolor("#1e1e24")
    fig.patch.set_facecolor("#18181b")

    rooms = tmap.rooms
    doorways = tmap.doorways
    curr_room = tmap.current_room

    if not rooms:
        ax.text(0.5, 0.5, "No Rooms Mapped Yet\nRun 'scan' to discover environment",
                color="#a1a1aa", fontsize=14, ha="center", va="center")
        ax.set_axis_off()
        fig.tight_layout()
        fig.savefig(out_file, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)
        return str(out_file)

    # Establish room centers if not set or all zero
    room_centers: Dict[str, Tuple[float, float]] = {}
    cols = max(1, math.ceil(math.sqrt(len(rooms))))
    spacing = 5.0  # meters between room centers

    for idx, (r_name, r_info) in enumerate(rooms.items()):
        rc = r_info.get("center_relative_m", [0.0, 0.0])
        if rc == [0.0, 0.0] and idx > 0:
            # Auto-layout in grid
            gx = (idx % cols) * spacing
            gy = (idx // cols) * spacing
            room_centers[r_name] = (gx, gy)
        else:
            room_centers[r_name] = (float(rc[0]), float(rc[1]))

    # Draw rooms
    room_w, room_h = 3.6, 3.6  # nominal room box size in meters
    for r_name, (cx, cy) in room_centers.items():
        is_current = (r_name == curr_room)
        edge_color = "#38bdf8" if is_current else "#52525b"
        fill_color = "#0f172a" if is_current else "#27272a"
        lw = 2.5 if is_current else 1.5

        rect = patches.FancyBboxPatch(
            (cx - room_w / 2.0, cy - room_h / 2.0),
            room_w, room_h,
            boxstyle="round,pad=0.1,rounding_size=0.2",
            facecolor=fill_color,
            edgecolor=edge_color,
            linewidth=lw,
            alpha=0.9
        )
        ax.add_patch(rect)

        # Room Title & Badge
        title = r_name.replace("_", " ").title()
        if is_current:
            title += " ★ (Robot Here)"
        ax.text(cx, cy + room_h / 2.0 - 0.35, title,
                color="#f8fafc" if is_current else "#e4e4e7",
                fontsize=11, fontweight="bold", ha="center", va="center")

        # Draw room objects
        objects = rooms[r_name].get("objects", [])
        for obj in objects:
            heading = obj.get("heading_deg", 0.0)
            dist = obj.get("distance_m", 1.0)
            label = obj.get("label", "object")
            obj_id = obj.get("id", label)

            # Object position relative to room center
            rad = math.radians(heading)
            ox = cx + dist * math.sin(rad)
            oy = cy + dist * math.cos(rad)

            # Clamp visual dot inside room box
            ox = max(cx - room_w / 2.0 + 0.3, min(cx + room_w / 2.0 - 0.3, ox))
            oy = max(cy - room_h / 2.0 + 0.3, min(cy + room_h / 2.0 - 0.6, oy))

            # Dot & Label
            color = "#f59e0b" if "chair" in label else ("#10b981" if "sofa" in label or "couch" in label else "#a855f7")
            ax.plot(ox, oy, marker="o", markersize=7, color=color)
            ax.text(ox, oy - 0.22, f"{obj_id} ({dist}m)",
                    color="#fcd34d", fontsize=7.5, ha="center", va="top")

    # Draw Doorway connections
    for d in doorways:
        u, v = d.get("from"), d.get("to")
        if u in room_centers and v in room_centers:
            ux, uy = room_centers[u]
            vx, vy = room_centers[v]
            is_passable = d.get("is_passable", True)
            line_color = "#34d399" if is_passable else "#ef4444"
            ls = "-" if is_passable else "--"

            ax.annotate(
                "",
                xy=(vx, vy), xytext=(ux, uy),
                arrowprops=dict(
                    arrowstyle="<->",
                    color=line_color,
                    lw=2.0,
                    linestyle=ls
                )
            )
            # Label doorway midpoint
            mx, my = (ux + vx) / 2.0, (uy + vy) / 2.0
            ap = d.get("aperture_width_m", 0.85)
            pass_str = f"{ap}m" if is_passable else f"{ap}m (BLOCKED)"
            ax.text(mx, my + 0.15, f"{d.get('id', 'door')}\n{pass_str}",
                    color=line_color, fontsize=8, ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="#18181b", edgecolor=line_color, alpha=0.85))

    # Configure axes & scale
    all_x = [pt[0] for pt in room_centers.values()]
    all_y = [pt[1] for pt in room_centers.values()]
    pad = 3.0
    ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
    ax.set_ylim(min(all_y) - pad, max(all_y) + pad)
    ax.set_aspect("equal")
    ax.grid(True, color="#3f3f46", linestyle=":", linewidth=0.6, alpha=0.5)
    ax.set_xlabel("X (meters, East)", color="#a1a1aa", fontsize=9)
    ax.set_ylabel("Y (meters, North)", color="#a1a1aa", fontsize=9)
    ax.tick_params(colors="#71717a", labelsize=8)

    # Title & Legend
    ax.set_title("Autonomous Floor Map & Object Landmark Model",
                 color="#f4f4f5", fontsize=13, fontweight="bold", pad=12)

    legend_elements = [
        Line2D([0], [0], marker='o', color='w', label='Chair', markerfacecolor='#f59e0b', markersize=7),
        Line2D([0], [0], marker='o', color='w', label='Sofa / Couch', markerfacecolor='#10b981', markersize=7),
        Line2D([0], [0], marker='o', color='w', label='Other Landmark', markerfacecolor='#a855f7', markersize=7),
        Line2D([0], [0], color='#34d399', lw=2, label='Passable Doorway'),
        Line2D([0], [0], color='#ef4444', lw=2, linestyle='--', label='Blocked Doorway'),
        Line2D([0], [0], color='#38bdf8', lw=2, label='Current Robot Location')
    ]
    ax.legend(handles=legend_elements, loc="upper right", facecolor="#27272a", edgecolor="#52525b",
              fontsize=8, labelcolor="#d4d4d8")

    fig.tight_layout()
    fig.savefig(out_file, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    logging.info("Rendered floor map saved to %s", out_file)
    return str(out_file)
