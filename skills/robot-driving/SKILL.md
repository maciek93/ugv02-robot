---
name: robot-driving
description: Observe, calibrate, map and drive the user's Waveshare UGV02 through its Mac/Pi suite, including manual PWM trials and approved LiDAR missions.
---

# Robot driving

Work in `/Users/maciejbargiel/Documents/Robot`. Read its `README.md`, `data/progress.json`, and the relevant workflow document before claiming a capability is operational. The user's current physical instructions and authorisation take precedence over historical experiments and older project guides.

## Current hardware and evidence

The six-wheel UGV02 has a Pi 5, ESP32 motor controller, forward USB/UVC camera, LD06 spinning LiDAR on a CP2102 USB serial adapter, and a forward VL53L5CX 8×8 distance sensor on an MCP2221A USB-to-I²C board. The LiDAR's USB lead is mini-USB to USB-A. On 22 October 2026 the manually started ROS container published live LD06 scans and repeated VL53L5CX range frames to the Mac tools. An observation-only SLAM run saved a provisional occupancy grid and pose graph; autonomous floor navigation has not passed physical acceptance. Do not use older assertions that these sensors are absent or that the sensor is a VL53L4CD.

Direct PWM 50 for five seconds previously moved forward 82 cm by tape with under 2 cm left drift. Opposing PWM 80 for 5.5 seconds produced a human-estimated 180° physical right turn, and a later supervised repeat with opposite PWM produced an observed smooth ~180° physical left turn. Those are single observed references, not repeatability. The ESP32 low-speed PID mode surged; use the direct-PWM path for continuous trials. Guardian telemetry remaps firmware `odl` to physical right and `odr` to physical left. The human measured a 0.25 m by 0.23 m chassis and LD06 centre about 0.075–0.085 m ahead of its centre, laterally centred. The LiDAR zero arrow points to chassis-right; four front-wall scans fitted a candidate yaw around −85.8°, but absolute wall alignment error is unknown. A later 7-second fixed-PWM turn reached only ~117° by live pose at 7.83 V. This showed fixed timed turns were unreliable; voltage alone is not a reason to disarm. A software-only adaptive PWM controller now raises or lowers power from measured wheel speed, but live closed-loop validation is pending. See `records/adaptive-motion-20261022.md`.

## Choose the right operating path

- For a live observation or manual floor calibration, use the existing camera-and-STOP page and `scripts/robot.py`/`scripts/chat_control.py`. In manual mode, a human-approved substantial PWM trial is continuous, not a sequence of tiny pulses. Record actual tape/floor-line observations separately from wheel counts. Use prior user authorisation within its stated bounds; avoid repetitive generic clearance questions.
- For LiDAR mapping and destination driving, read `docs/lidar-mapping.md`. The ROS sidecar provides raw scan and depth-grid tools and an observation-only measured map. The provisional map cannot authorise motion. The old `scripts/llm_driver.py show_map` remains only a nominal semantic room diagram. Codex may preview and submit room-labelled waypoints through approved directed doorways, or issue bounded direct distance/angle commands, within one approved metric mission after an authorised navigation profile exists. Nav2 is the default destination planner. Keep the map frame, metres/radians, source timestamps and version explicit.
- `mapping` mode requires a version-2 measured LiDAR/motion profile and an approved bounded mission. The Pi guardian alone writes ESP32 commands and enforces STOP, mission expiry/travel quotas, scan freshness and short motor leases. Ordinary route segments and direct moves inside that mission need no separate approval. A stop, pause, fault or restart removes movement authority; start a new mission to resume. Do not install a profile from software-only or single-trial evidence.
- The older camera-only `explore` implementation has its own version-1 profile and is not proof that the LiDAR stack works. Read `docs/exploration-v1.md` only when using that mode.

Start with `.venv/bin/python scripts/start_suite.py`, then check `.venv/bin/python scripts/robot.py status`. The user has supplied the Pi password in this chat and explicitly authorised using it directly; do not repeatedly ask them to enter it. The viewer at `http://127.0.0.1:5001/` retains camera and STOP only. When observation ends, pause/end the session so camera and vision resources release. Never bypass the Pi guardian with raw serial motor commands to obtain movement.

For repeated physical measurement procedures, read [references/experiments.md](references/experiments.md). Update `data/progress.json` and the mission record after meaningful evidence, including failed trials. Saved maps can inform a new mission, but no saved pose or movement authority is restored after restart; localise against current scans first.

If you notice a software bug or think of a practical improvement while using the robot, tell the human what you found, why it matters, and what you recommend. Fix issues within the authorised task when feasible, and still report the finding; suggest larger changes rather than silently expanding the task.
