# LD06 metric mapping and Codex tools

Updated 22 October 2026. The LD06 and forward VL53L5CX are connected to Pi USB ports. The LD06 USB lead is mini-USB to USB-A. The VL53L5CX uses an MCP2221A USB-to-I²C adapter; it is **not** the originally listed VL53L4CD. The camera remains USB/UVC below neither sensor's control interface.

## Verified now

- The LD06 CP2102 port is `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`. Its raw zero arrow points to chassis-right. Four wall-aligned scans gave a **candidate** chassis-to-LiDAR yaw of about −85.8°; the absolute error of the human wall alignment is not yet measured.
- The MCP2221A enumerates as `04d8:00dd` and accesses VL53L5CX at I²C address `0x29`. On 22 October, a corrected driver initialized the sensor and returned repeated 8×8 frames. ST's current ULD treats MCU boot register `0x0006` bit 0 as success; the older Python port incorrectly waited for zero. The 32-bit word swap is needed to align range and status zones.
- ROS 2 Jazzy runs in a manually started Pi container. An observation-only profile produced a provisional occupancy grid and pose graph from live scans and wheel odometry. The saved map is `data/metric_maps/observation-df8e9ddae0281acc1c7242e7/`; it cannot authorise navigation. The `/tof/grid` topic delivered repeated 64-zone frames. A front-wall scene gave 61 status-valid zones and a median 1.21 m distance, close to the LiDAR wall fit at 1.175–1.176 m; zone direction remains untested.
- The human measured a 25 cm × 23 cm chassis, with the LD06 laterally centred and 7.5–8.5 cm forward of the chassis centre. One tape-measured direct-PWM forward run was 82 cm in 5 s at PWM 50. Later LiDAR-aligned forward legs were approximately 0.397 and 0.393 m in 3 s at PWM 50. Fixed-PWM turns varied markedly as voltage fell: a 7 s turn gave about 162.5° by scan alignment at 8.54 V, then a later 7 s mapping turn gave about 117° by live pose at 7.83 V. These are scene-specific estimates, not a precise voltage-response curve.

## Architecture

`ugv-supervised` on the Pi remains the sole ESP32 serial writer. The ROS 2 Jazzy sidecar publishes `/scan`, `/tof/grid`, `/odom`, map and localisation; SLAM Toolbox supplies the metric occupancy grid and pose graph, and Nav2 is the default planner/controller. The Mac gateway relays Codex's high-level requests to the ROS bridge over the existing SSH connection. The Pi guardian independently checks mission identity, short motor leases, advancing LiDAR scans, viewer connection and the cumulative mission budget. PWM setpoints are latched between renewals; lease renewal does not pulse the motors. The bridge uses saved speed-at-PWM figures only as an initial power guess, then changes each wheel's PWM from observed encoder speed. Voltage itself is not an inhibit threshold. Direct turns finish from accumulated map heading when available. The Pi charges the travel budget using measured wheel travel plus a reserve for unobserved movement and fallback stopping. See [adaptive motion](../records/adaptive-motion-20261022.md).

The legacy `show_map` is a **semantic room diagram** with nominal geometry. `scripts/metric.py view-metric-map` is a crop of measured ROS occupancy cells, including unknown cells and a map version. `view-lidar-scan` returns timestamped raw ranges and a polar image; `view-depth-grid` returns VL53L5CX zone distances and status. A distance is usable only when its own zone status is valid. Sensor-only results report `map_version: null` until SLAM is running.

The Mac validates each route's room labels and directed doorway sequence against the approved mission. These labels are semantic annotations; until surveyed room boundaries are tied to map coordinates, they are not an independent geometric proof that a route stays inside a named room. The occupancy-grid footprint check remains the metric clearance gate.

## Startup and inspection

The ROS container is manually started; its boot service is disabled. Without an installed measured `metric-profile.json`, it starts sensors and the map/scan bridge only. It cannot authorise navigation. The Mac camera-and-STOP page stays at `http://127.0.0.1:5001/`.

```sh
.venv/bin/python scripts/start_suite.py
ssh -S .runtime/ssh-control ugvpi@ugvpi.local 'sudo systemctl start ugv-ros2'
.venv/bin/python scripts/metric.py view-lidar-scan
.venv/bin/python scripts/metric.py view-depth-grid
```

Once a measured profile is installed and the sidecar restarted, SLAM Toolbox and Nav2 start. To create an approved metric mission:

```sh
.venv/bin/python scripts/chat_control.py session start --mode mapping
.venv/bin/python scripts/metric.py propose config/metric-mission.json
# Review the proposal with the human and record their actual approval:
.venv/bin/python scripts/chat_control.py approve-metric-mission MISSION_ID --human-instruction 'ACTUAL WORDS'
.venv/bin/python scripts/metric.py start MISSION_ID
.venv/bin/python scripts/metric.py view-metric-map
.venv/bin/python scripts/metric.py preview-route route.json
.venv/bin/python scripts/metric.py follow-route route-selection.json
```

The last command takes a JSON file path, like all route commands. `navigate-to` accepts `{"target":{"x":1.0,"y":0.0,"yaw":0.0,"room":"A"}}` in the ROS `map` frame and remains within the current recorded room. `preview-route` accepts `{"waypoints":[{"x":...,"y":...,"yaw":...,"room":"A"}]}`. Every waypoint names an approved room; the first waypoint in another room also names an approved directed `via_doorway`. `follow-route` accepts a JSON file containing the returned `preview_id`. After reaching a cross-room route, record the actual transition with `annotate` and its reached goal ID before further room-scoped travel. `drive-distance` takes positive metres up to 2; `turn-angle` takes signed degrees up to 360. Use `status`, `capture-view`, `annotate`, `pause`, `stop`, `finish` and `report` for mission progress. STOP on the page immediately cancels mission motion. A restart never restores movement authority.

## Calibration and operational gate

No checkerboard is required. Measure chassis length, width, axle track, LD06 x/y/yaw, physical forward and both turn directions, wheel encoder signs/units and stopping distance on each intended surface. The speed-at-PWM and minimum smooth PWM values are starting estimates for the feedback controller; they need not remain accurate as voltage changes. Geometry, encoder direction/scale, the LiDAR transform and a conservative stopping envelope still matter. For LiDAR yaw, physically align the chassis with a straight wall and save a raw scan. On this mounting, the front wall appears in the raw left sector, so run `.venv/bin/python scripts/fit_lidar_wall.py SCAN.json --side front --scan-sector left`. The tool fits a *candidate* yaw plus fit error; compare several aligned scans and account for physical wall-alignment error. Record physical distance separately from encoder travel; encoder counts alone are not travel evidence. Do not install a profile until those values and the three-second ESP32 fallback distance are evidenced.

The profile format is `version: 2`, `kind: lidar-mapping`, with configuration, sensor serial, surface, chassis geometry, LiDAR transform, corrected encoder signs/scale, motor polarity, calibrated forward and turn PWM/rates, speed/turn upper bounds, fallback stop distance, margin and evidence text. `scripts/chat_control.py metric-profile FILE --human-instruction ...` installs it through the operator bridge; the Pi validates it and cancels existing authority. Restart the ROS sidecar afterwards so Nav2 gets the measured footprint.

Physical navigation is pending: repeatable forward/left/right calibration, LiDAR yaw absolute alignment, stopping distance, map alignment, obstacle stopping, doorway traversal and two completed two-room surveys have **not** been demonstrated. Code or a successful sensor read must not be presented as floor-navigation acceptance.

`scripts/reuse_metric_map.py MISSION_ID X Y YAW_RAD` selects a saved SLAM pose graph and a proposed starting pose for a new, disarmed localisation attempt. It restarts the ROS sidecar; an old mission is not restored. Check the current scan against the map and obtain a new mission approval before moving. The saved map and pose graph are under `~/robot-maps/MISSION_ID/` on the Pi, while camera checkpoints and reports are under `data/metric_missions/MISSION_ID/` on the Mac.
