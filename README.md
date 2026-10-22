# UGV02 robot suite

## Public repository contents

This repository contains the Mac/Pi/ROS source, robot-driving skill, setup notes,
checkerboard artifact, selected experiment telemetry, and numeric calibration
evidence. **Raw camera images, the home occupancy map and pose graph, private
session logs, credentials, downloaded model weights, and vendored third-party
checkouts are intentionally local-only.** The selected numeric records were
reviewed for credentials before publication. Some historical records refer to
local-only images or maps; those references describe the original experiment,
not files required to run the code. Download model assets using the documented
scripts and their pinned manifests before running the relevant vision mode.

## Current hardware and LiDAR mapping work (22 October 2026)

The LD06 LiDAR and forward-facing **VL53L5CX** 8×8 distance sensor are now physically
connected through separate Pi USB adapters. The LD06 streams scans through its
CP2102 adapter. The VL53L5CX, via MCP2221A USB-to-I²C, now initializes and returns
repeatable 8×8 frames after correcting an obsolete MCU-boot check and zone byte
ordering in the Python driver. See [the LiDAR mapping workflow](docs/lidar-mapping.md).

The manually started ROS 2 Jazzy sidecar publishes live LD06 scans and
VL53L5CX grids to the Mac tools. An **observation-only** SLAM run produced a
provisional occupancy grid and pose graph, saved locally under
`data/metric_maps/observation-df8e9ddae0281acc1c7242e7/`. It does not authorise
autonomous driving. The direct-PWM navigation bridge now adjusts motor power from
observed wheel speed; measured distance and LiDAR heading close direct moves,
and the Pi charges mission travel against observed wheel motion. These software
changes passed replay checks but have not had a live closed-loop drive. See
[adaptive-motion evidence](records/adaptive-motion-20261022.md).
**No measured LiDAR/motion navigation profile is installed, and autonomous floor
navigation is not operational.** The
camera-and-STOP page and existing manual drive remain the verified operating path.
The older camera-only sections below are dated implementation history; their
claims that no ranging sensor is installed are superseded by this update.

## Camera-only exploration v1 (software implementation)

V1 adds mission-level chat approval, a Pi-enforced mission budget, a dedicated
Mac navigation worker, floor segmentation and visual/encoder agreement checks,
conservative swept-footprint mapping, and persistent survey reports. The
implementation is **not yet physically validated or operational**. Manual floor
trials on 14 October identified surging in the ESP32's low-speed PID mode and
established one smooth, tape-measured forward run and a measured balanced turn
using direct PWM; see the live movement record below.

Start with [the v1 workflow](docs/exploration-v1.md). It includes deployment,
stationary calibration, evidence gates, mission commands, replay and acceptance.
Print [the calibration checkerboard](output/pdf/checkerboard-20mm.pdf) at 100%.
The [original lidar decision](records/navigation-hardware-plan-20261013.md) is saved;
the purchased hardware is now being integrated through the separate metric stack.

The source backup is `records/backups/pre-explore-v1-20261013.tgz`. Tests and saved
image benchmarks support software validation only. Exploration remains locked
until a reviewed versioned camera/motion/perception/watchdog/latency profile is
installed. Live latency and the two-room demonstrations remain pending.

Local verification: 67 tests pass, Python compilation and dependency checks pass.
The three-model saved-image benchmark measured 58 ms warm median / 62 ms warm
maximum on this Mac; cold inference exceeded 500 ms. These figures exclude camera,
network, geometry and motor delivery. Details: [implementation evidence](records/explore-implementation-20261013.json).

The sections below preserve the previous physically verified state and legacy
bench/calibration/tour workflow. Use the v1 guide for `explore` missions.

Updated 14 October 2026, Europe/London. Pi 5 + UGV02 ESP32 chassis, USB/UVC camera,
Mac M5/32 GB for vision and reasoning. No ranging sensor or arm is installed.

## Current verified state

Stage: one **smooth measured manual forward run**. Direct PWM 50/255 for 5 seconds
travelled 82 cm by human tape measurement, with less than 2 cm left drift; the
encoder fields changed -85/-84. A balanced PWM 80/80 right turn took 5.5 seconds
for a human-confirmed 180° against a floor-panel line. An earlier full-turn test
took 11.1 seconds and ended about 5–10° past 360°. These are first references,
not repeatability or autonomous navigation calibration. Earlier low-speed PID runs
surged and veered, even after reducing setpoint writes to one per second. Turn
tests showed the firmware's `odl` field follows the **physical right** wheels and
`odr` the **physical left** wheels; software still uses the old labels, so do not
use those counts for autonomous yaw or steering until the mapping is corrected.
See the [live motion record](records/live-motion-20261014.md).
The earlier 2.01-second trial with the old polarity moved backwards about 125 mm.
Floor calibration, obstacle
validation, room transitions and return navigation have **not** been verified.
Live status is authoritative; read `data/progress.json` and the experiment records.

The page at http://127.0.0.1:5001/ contains only the camera and STOP button.
Codex controls sessions and requests approvals in chat. Discuss any UI expansion
with the user first. `ugv-web` is retired/masked; do not resurrect its old controls.
`ugv-supervised` is manually started and boot autostart is disabled. Idle/paused
states release the camera and stop Mac vision; the camera remains USB-powered.

Verified: fresh UART telemetry after boot; power flags 0x0 at checks; live camera,
Core ML Depth Anything V2 Small and YOLO26n in guarded modes; smooth forward
movement and Stop in one direct-PWM manual trial, and physical right turns with
both sides counter-rotating at PWM 80. The opposite turn direction is unverified.
Normal guarded pulses stopped in roughly 250–261 ms.

Web STOP cancelled pending travel after 0.5 of an approved 1.0 seconds; replay
returned 423. Camera/perception failure each cancelled a sequence after its first
pulse. Viewer loss cancelled travel and released camera/vision. End/restart
passed with camera off and no restored approval. Twenty-three simulated control tests
passed; these do not prove floor safety. Clean live shutdown was about 0.19 s,
including SSH, with no forced kill. Selected evidence: `data/experiments/`,
`records/supervised-idle-checks.json`, `records/supervised-active-shutdown.json`.

Installed discoverable skill: `~/.codex/skills/robot-driving/SKILL.md`.
Its instructions distinguish manual driving from the still-unvalidated exploration mode.

## Startup and operator workflow

From this project directory:

```sh
.venv/bin/python scripts/start_suite.py
# If AUTHENTICATION_REQUIRED, enter the Pi password only in the opened Terminal,
# then run startup again. No passwords belong in chat, files or command arguments.
.venv/bin/python scripts/robot.py status
.venv/bin/python scripts/chat_control.py session start --mode bench
# Human-supervised large floor movements use --mode manual.
# Guarded floor trials use --mode calibration; tour remains locked until validated.
.venv/bin/python scripts/robot.py observe --save data/observations/current.jpg
.venv/bin/python scripts/robot.py propose pulse_forward 0.25
# Only after an actual human approval for the exact trial/approved sequence:
.venv/bin/python scripts/chat_control.py approve PROPOSAL_ID --human-instruction 'EXACT HUMAN INSTRUCTION'
.venv/bin/python scripts/robot.py execute PROPOSAL_ID
.venv/bin/python scripts/robot.py stop
.venv/bin/python scripts/chat_control.py session pause --mode bench
.venv/bin/python scripts/robot.py end
```

Start/resume open the camera but do not move motors. Bench/calibration/tour/explore
modes load vision and enforce the configured camera, perception and viewer checks.
Manual mode leaves those checks off while keeping the camera+STOP page, a fixed
command deadline and short Pi motor leases. In guarded modes, the viewer heartbeat
expires after 0.6 s; closing or hiding it stops movement. Reopen the page
and explicitly resume. End stops the
Pi service and closes the app forwarding; the separately opened authenticated
Terminal SSH login remains available until the user closes it.

`robot.py` has no approval operation. The trusted chat bridge records human
approval provenance and uses a separate operator credential. The Pi checks exact
proposal/session identity, freshness, single use, bounded quota and current
conditions. This is **not cryptographic verification of chat authorship**; Codex
must never invent approval text or reuse cancelled/out-of-scope authorisation.

## Bounds and tests

Guarded requested wheel speeds remain +/-0.05 m/s. Normal motor leases
are at most 0.25 s, checked every approximately 20 ms; no command backlog exists.
Bench sequences may contain 1.0 s total motor time, with stops and observations
between pulses. Explicitly chat-approved forward calibration sequences allow 0.05–2.0 s total motor time per approval, divided into leases of at most 0.25 s. Uncalibrated turns remain limited to 0.25 s.
The `continuous_forward` action allows up to 30 s in calibration or manual mode.
Manual `continuous_forward` requests 0.10 m/s through the ESP32 speed PID; it surged
in live tests. Manual `pwm_forward` applies direct PWM 50/255, bypassing that PID.
One five-second trial travelled a measured 82 cm smoothly. The manual
`pwm_spin_right` action counter-rotates both sides at PWM 80; 5.5 seconds gave a
human-confirmed 180° right turn, and 11.1 seconds gave about 365–370°. At PWM 50,
one side stalled during turns; at asymmetric 80/50, the other side stalled. These
values need repetition, battery/surface scoping and corrected physical side labels
before navigation can use them. Manual continuous left/right actions are bounded
to 10 s each, but their physical directions remain unverified. Motor
commands are refreshed about once per second for the ESP32 heartbeat while Pi
leases renew at most every 250 ms. STOP or a missed lease ends the command. Manual
mode does not enforce camera, vision, viewer or telemetry vetoes.
Tour limits are 50 cm forward or 15 degrees turning; no blind reversing. Physical
speed/distance are uncalibrated. Software timing is not a hard realtime guarantee;
the ESP32 fallback remains relevant if the Pi/UART cannot send Stop.

In guarded modes, missing/stale telemetry (>0.75 s), camera or perception (>0.5 s), bad image quality,
connection loss and configured hazard vetoes prevent motion. Bench depth/person
checks are in shadow mode because wheels are supported; this mode must never be
used for floor driving. Floor calibration blocks detected people/pets but has no
validated depth-distance safeguard yet. The existing relative-depth cue is not
metres, and its usefulness across scenes must be measured before enabling tour.

```sh
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/experiment.py pulse_forward 0.25 --label TRIAL --context clear-floor --approval 'EXACT HUMAN APPROVAL'
.venv/bin/python scripts/measurement.py data/experiments/TRIAL_DIR --travel-mm NUMBER --braking-mm NUMBER --notes 'HUMAN MEASUREMENT'
```

An experiment saves before/after JPEGs, observations, telemetry, approval and
result. It separates software stopping from human physical verification. Floor
experiments require a recorded fixed camera configuration. Measurements are
scoped to that configuration and surface. There is no continuous video recording.
`measurement.py` creates a draft, never activates a tour profile automatically.

The misnamed `records/floor-initial.jpg` was captured **on the book** following a
corrected human message. It is not floor calibration evidence. Historical setup,
older interfaces and dependency fixes are preserved in
`records/setup-history-20261012.md`; its startup instructions are retired.

## Services, logs and recovery

Pi service: `ugv-supervised`, source `~/ugv-supervised`, using the existing reviewed
`~/ugv_rpi/ugv-env` packages. Camera runs in a separate process; motor supervision
imports no vendor camera, audio, arm or gimbal code. Mac vision runs in a separate
process and its failure cannot extend a motor lease. Local ports: Mac 5001 viewer
and scoped agent API, Mac 5002 SSH forwarding, Pi 5000 loopback-only controller.
No Jupyter, hotspot, model daemon or boot camera service was enabled.

```sh
ssh -S .runtime/ssh-control ugvpi@ugvpi.local 'systemctl status ugv-supervised --no-pager'
ssh -S .runtime/ssh-control ugvpi@ugvpi.local 'journalctl -u ugv-supervised -b -n 100 --no-pager'
ssh -S .runtime/ssh-control ugvpi@ugvpi.local 'sudo systemctl stop ugv-supervised'
# Deployment, only while stopped, replaces reviewed service/source; camera remains off:
.venv/bin/python scripts/deploy_supervised.py
```

Camera orientation: `~/.config/ugv-supervised/camera.json` on the Pi contains
`rotation_degrees: 180`; rotation is applied before camera output and perception.
Older unrotated observations are not floor calibration evidence.
Motor direction: Pi `~/.config/ugv-supervised/motors.json` contains `motor_polarity: [-1, -1]`, based on the measured reversed forward motion. The human confirmed physical forward movement in a subsequent one-second trial; straight tracking and turn conventions remain unverified. UART feedback retains the firmware's raw signs, so forward travel currently gives negative `odl`/`odr` readings. Navigation profiles must explicitly convert those signs.

Mac logs: `.runtime/mac.log`; metadata actions: `records/supervised-actions.jsonl`;
experiment results: `data/experiments/*/trial.json`. Credential files under
`.runtime` and Pi `~/.config/ugv-supervised` are private, excluded from Git, and
must not be printed or included in data bundles. If STOP cannot reach the robot,
its lease expires; the chassis power switch is the physical fallback. Recover by
reconnecting SSH and restarting disarmed, observing anew and obtaining new approval.
Do not bypass blocked guards or flash firmware to make a test pass.

## Hardware inventory and later stages


Pi: Raspberry Pi 5 Model B Rev 1.1, 4 GB; Debian 12 Bookworm/aarch64;
kernel `6.12.109+rpt-rpi-2712`. SSH user `ugvpi`; host `ugvpi.local`;
IPv4 `192.168.86.45` (DHCP, may change). Existing dialout/video/i2c membership
and noninteractive sudo work. Initial temperature 33.4 C, later 44.4 C;
checked power flags `vcgencmd get_throttled` were `0x0`. This does not prove
power stability under sustained motor load.

Camera: USB `1bcf:c001`, UVC driver, video0 capture/video1 metadata. Select
`/dev/v4l/by-id/usb-USB_USB-video-index0`, not Pi codec/ISP video19–35. MJPEG
640x480 at 30 requested fps; measured delivery about 13 fps. A real independent
image, 90-frame capture, uncached JPEG snapshot, and two simultaneous 10-second
streams (127/128 decoded frames each) passed. User identified themselves in live
video. USB camera detection cannot rely on the word “Camera” in lsusb: this
camera's device name is simply “USB”. One reader now shares frames among clients.

GPIO UART: `/dev/ttyAMA0`, 115200 baud, GPIO14=TXD0 / GPIO15=RXD0 (physical
8/10). Pi 5 `/dev/serial0 -> ttyAMA10` belongs to the separate debug connector.
A T=130 telemetry request received T=1001 feedback. We proved both directions
by pausing telemetry with T=131/cmd=0, checking a quiet window, requesting T=130,
and restoring T=131/cmd=1. No firmware flashing or calibration was performed.
User confirmed the 40-pin cable is connected and Pi USB-C is empty. Functional
UART is proven; the extension's exact product and remaining pin mapping are not.

Battery: three user-supplied 3000 mAh cells in series: nominal 11.1 V/3000 mAh.
Readings rose from 11.49 V to about 11.9 V during setup; user reports a red
charger indicator. Waveshare's 12.6 V/2 A charger specifies red while charging,
green at completion (also green with no load). Current telemetry does not give
charge percentage, individual cell voltages, charge current or time remaining.


VL53L4CD and Ozzmaker adapter remain disconnected. Verify physical header access,
extension orientation/continuity, 3.3 V, SDA, SCL and ground before wiring. Do not
assume the ESP32 bus is the Pi bus. No arm configuration or extra PSU is installed;
do not connect USB-C power alongside chassis GPIO power without verified isolation.

## Sources and reproducibility

- Vendor and OS setup commits/commands: `records/setup-history-20261012.md`.
- Mac resolved packages: `records/mac-python-freeze.txt`.
- Mac measured model latency: `records/mac-vision-benchmark.json` (saved image).
- Depth model: https://huggingface.co/apple/coreml-depth-anything-v2-small
- Detector: https://docs.ultralytics.com/models/yolo26
- Chassis: https://www.waveshare.com/wiki/UGV02
- Vendor source: https://github.com/waveshareteam/ugv_rpi
