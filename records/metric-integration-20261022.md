# Metric navigation integration, 22 October 2026

The Pi is reachable through the existing SSH control connection. ROS 2 Jazzy was built in an Ubuntu 24.04 arm64 container and runs as the manually started `ugv-ros2` service. Both that service and `ugv-supervised` are active; boot autostart remains disabled. The Pi guardian is idle, no measured metric profile is installed, and no movement command was sent during this integration.

## Live evidence

- The pinned LD06 ROS driver compiled after adding the missing pthread header. It opened the CP2102 at 230400 baud, reported normal communication, and published `/scan`. The Mac tool returned 456 beams and a polar image on the final deployment. The Pi guardian saw an advancing scan sequence with a source timestamp about 0.01 s old.
- The MCP2221A/VL53L5CX ROS node initialized and published repeated `/tof/grid` frames. The Mac tool returned 64 distances, status values and 8 status-valid zones in the final sampled frame. Earlier stationary frames had only 2–6 valid zones. Physical target-distance and zone orientation have not been checked.
- The camera-and-STOP page returned HTTP 200 at `http://127.0.0.1:5001/` after both services restarted. The Pi reported `moving: false` and `profile: null`.
- Over a two-second live sample, LD06 scan sequence advanced by 21 and VL53L5CX frame sequence advanced by 10; the final source ages at Mac readout were about 0.12 s and 0.15 s, respectively. This checks sensor-to-tool freshness, not capture-to-control latency under active Nav2 motion.
- `view-metric-map` correctly returned `No measured occupancy grid`; the old semantic room diagram has not been relabelled as a LiDAR map.

## Software checks

- 117 unit and replay tests passed, including mission expiry/quota, duplicate renewal, stale or frozen LiDAR timestamps, route footprint and unknown-cell checks, directed doorway scope, and recovery of a known LiDAR wall yaw in simulation. The test run logs an intentional simulated UART fault.
- Python compilation and ROS YAML parsing passed.
- An isolated container with a simulated profile started SLAM Toolbox and the Nav2 nodes after correcting the Nav2 launch argument `use_composition` to `False`. That container had no device mounts, keys or odometry, so its sensor and transform warnings were expected. It did not produce a usable map or exercise motors.

## Remaining release evidence

Confirm the LiDAR yaw against a physical wall reference after the measurements below; measure stopping distance and encoder scale across repeat trials, and check VL53L5CX zones against targets at known distances. Only then install a version-2 profile and test live SLAM/odometry alignment, Nav2 route following, obstruction stopping and connected-room surveys. No autonomous navigation outcome is claimed yet.

## Later supervised calibration on 22 October

The human measured a 0.25 m by 0.23 m chassis. The LD06 is laterally centred, with its centre 0.04–0.05 m behind the front edge, giving `lidar_x_m` in approximately `[0.075, 0.085]` from the chassis centre. The arrow indicating raw LiDAR zero points to the **robot's right**. A flat object held directly ahead altered raw scan returns around 0–100°, confirming that raw +90° is approximately chassis-forward; this is a coordinate-frame rotation in addition to a small mounting skew.

With the chassis front edge positioned parallel to a wall, four clear LD06 scans fitted the front wall in the raw left sector. The fitted chassis-to-LiDAR yaw was **−85.86°, −85.87°, −85.74°, −85.73°** (about −1.497 rad). The wall distance was 1.175–1.176 m with 9–11 mm fit RMS. The VL53L5CX simultaneously produced 61 status-valid zones, with median valid range 1.21 m; the nearest zones were about 0.62 m, so zone direction and floor/cabinet returns remain to be resolved. Source scans: `data/observations/ld06-front-wall-aligned-{1,2,3,4}-20261022.json`; target scan: `data/observations/ld06-front-target-20261022.json`; distance grid: `data/observations/vl53l5cx-front-wall-20261022.json`. The repeatability describes scan fitting, while absolute yaw still inherits the human wall-alignment error.

Two 5.5-second balanced PWM 80 left turns completed. The human observed the second as a smooth approximately 180° physical left turn with both sides moving; the first endpoint was not observed. See `records/live-motion-20261014.md` and the linked experiment directories. The battery dipped to 8.76 V during the second turn, so further motor trials were paused pending charging. Stopping distance, encoder repeatability, and a loaded metric profile remain outstanding.
