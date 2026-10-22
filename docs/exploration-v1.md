# Camera-only supervised exploration v1

## Status and responsibilities

This implementation adds bounded missions, floor perception and visual/encoder
tracking, a local map, continuous short-leg control, checkpoint records and
survey reports. It has **not** completed physical acceptance. The last recorded
floor trial reported no motion despite encoder changes. Direction, reliable
travel, stopping, camera geometry and held-out obstacle performance remain
unverified. Exploration cannot start without a versioned, reviewed profile.

Mac: local model process, geometry and map; separate 20 Hz navigation thread;
Codex chooses numbered goals. Pi: session, authority, freshness checks, 100 ms
renewal deadline and <=250 ms motor leases. ESP32: motor control and its own
command-loss fallback, which must be verified physically. The Pi does not know
which room it occupies; room/doorway scope is a Mac navigation constraint.

Use prepared connected rooms with drops excluded, people/pets outside the driving
area and loose cables removed. The forward camera has blind areas. Flat floor
projection is an estimate. Relative depth is never metres. Unknown cells block
movement. Map clearance ages out after the reviewed `clearance_max_age_s` (draft
default 5 s, maximum 30 s). A slow leg may stop when its rear clearance expires;
any longer retention must be validated for the prepared, static scene. People/pet detections are supplementary. No reverse, full SLAM,
unattended driving, automatic return or boot startup is provided.

The first operational demonstration is two supervised two-room surveys, each
ending stopped at a recognised checkpoint with no contact and an accurate report.

## Installation and rollback

The existing Python environment has been extended with Transformers, ReportLab
and pypdf. `records/mac-python-freeze-explore.txt` records resolved versions. The
floor model is downloaded locally, pinned in `models/explore-manifest.json` with
revision and hashes of weights, preprocessing and label configuration. Runtime
verifies those bytes before exploration. To restore the pinned model assets:

```sh
.venv/bin/python scripts/prepare_explore_models.py
```

A new model manifest requires profile revalidation. No model is downloaded during
robot observation. Model workers are loaded only for an explicit session and
released on pause/end. The browser remains camera + STOP only.

Deploy only after reviewing code and stopping any active session:

```sh
.venv/bin/python scripts/deploy_supervised.py
.venv/bin/python scripts/start_suite.py
.venv/bin/python scripts/robot.py status
```

Deployment stops the Pi service before replacing its source, preserves existing
credentials and adds a separate navigation credential, then starts idle/disarmed.
Gateway protocol 3 prevents accidentally reusing the old gateway. No firmware
flashing or wiring changes are included. An authenticated SSH connection is
required; enter passwords only in Terminal when prompted by existing startup.

A source snapshot from before this implementation is in
`records/backups/pre-explore-v1-20261013.tgz`. It excludes credentials and models.
For rollback, stop the suite, inspect/extract that archive to a separate directory,
restore reviewed source, and deploy while stopped. Do not restore authority or
mix old and new profile formats.

## Motion evidence comes first

```sh
.venv/bin/python scripts/diagnose_motion.py data/experiments/20261013-150507-15f6de
```

This is offline: it separates command, raw encoder deltas and human evidence.
The changed raw counts are not accepted as proof of travel. Check physical
forward/turn signs, scale, possible encoder resets, reliability at the existing
0.05 m/s requested ceiling, and stopping. Do not raise speed to overcome a failed
trial without a separately reviewed change.

Use the existing bounded bench/calibration experiments after the human confirms
current setup and readiness. Record measurements, surface, polarity and actual
command durations. Obtain three repeatable small forward and both-direction turn
trials before activating an exploration profile. The later 10/25/50 cm trials
are operational acceptance, not prerequisites for collecting stationary data.

ESP32 command-loss testing is a separate, explicitly prepared supported-wheel
experiment. The vendored source has a 3000 ms default, not evidence of the actual
running firmware. Stopping the Pi service normally sends Stop and therefore does
NOT prove firmware timeout. Do not kill/suspend a moving guardian on the floor.
Record observed cessation of rotation after actual command loss and the test
setup; derive a conservative fallback travel bound from measured upper speed and
observed upper fallback time (`fallback_timeout_s`) plus braking. Profile
validation enforces that minimum distance bound. If fallback cannot be established, exploration
remains locked. No automated fault injection against live motors is installed.

## Camera calibration

Print `output/pdf/checkerboard-20mm.pdf` on A4 landscape at Actual Size/100%,
disable Fit to page, verify the 100 mm ruler and 20 mm squares, and mount flat.
There are 10x7 squares / 9x6 internal corners. No navigation markers are needed.

With the robot stationary and camera fixed, start a calibration observation
session. The operator must be ready before capture:

```sh
.venv/bin/python scripts/chat_control.py session start --mode calibration
.venv/bin/python scripts/calibrate.py capture data/calibration/views
.venv/bin/python scripts/calibrate.py intrinsics data/calibration/views --output data/calibration/intrinsics.json
```

Capture accepts 20 varied sharp views by default and times out after five minutes.
Move/tilt the board across the frame; robot motors remain untouched. Intrinsics
require >=15 diverse views, stable focus/settings, RMS <=1 px and per-view error
<=1.5 px. Camera capture requests autofocus off and records actual focus readback;
unsupported focus locking blocks release instead of assuming success. Calibration
binds to the USB product and serial (or physical USB port when no serial is
available), resolution, rotation, focus and named mounting configuration. A
serial-less camera swapped for an identical unit cannot be detected automatically;
recalibrate after any camera replacement or mounting change.

Place the board on the floor with its FAR EDGE away from the robot and its edges
aligned to the chassis axes. Measure from the chassis centre: x forward to the
near board edge; y left to the board's left edge. Save RGB plus metadata:

```sh
.venv/bin/python scripts/calibrate.py snapshot data/calibration/floor.jpg
.venv/bin/python scripts/calibrate.py floor data/calibration/intrinsics.json data/calibration/floor.jpg --near-edge-m MEASURED_X --left-edge-m MEASURED_Y --output data/calibration/camera.json
```

The first floor command deliberately produces a numbered preview and no accepted
calibration. Inspect it: corner 0 must be near-left, 8 near-right, 45 far-left.
Use `--reverse-rows` / `--reverse-columns` to correct detection orientation, then
repeat with `--orientation-confirmed` only once the labels are right.

```sh
.venv/bin/python scripts/calibrate.py draft-profile data/calibration/camera.json --configuration YOUR_ID --surface wood --output data/calibration/explore-profile.json
```

Fill the measured footprint/track width, encoder scale/signs, polarity, blind
area, upper movement rates, braking and fallback bounds. Defaults for perception
thresholds are only candidates; derive reviewed limits from calibration scenes
and test them on separate held-out scenes. All validation flags start false.

## Perception and latency validation

Use a complete draft profile without enabling exploration. Stationary shadow
capture runs floor segmentation, geometry, depth and detection and saves RGB,
metadata, overlays and geometry. It does not move or grant movement authority:

```sh
.venv/bin/python scripts/explore_shadow.py data/calibration/explore-profile.json data/validation/session-a --frames 100
```

The Pi-clock probe measures capture-to-processing-to-network-return latency.
Warm-up is recorded separately; require at least 95 warm samples below 350 ms.
This does not prove physical motor delivery or stopping. Test motion delivery
separately under the measured lease envelope. The existing 500 ms camera/vision,
750 ms telemetry and 600 ms viewer limits are unchanged.

Copy the generated dataset draft, assign calibration/held-out splits BEFORE
choosing thresholds, and label each frame `clear`, `blocked` or `warmup` with
human evidence. Each sequence needs a category. Include furniture, chair-leg,
doorway, reflective and clear-floor sequences; preserve failed cases. Use fresh
sequences after moving props or repositioning the camera; no automatic driving
is performed to create validation data. More varied motion sequences can be
recorded during separately approved calibration experiments.

```sh
.venv/bin/python scripts/validate_exploration.py replay data/calibration/explore-profile.json data/validation/held-out.json --output records/explore-perception-validation.json
.venv/bin/python scripts/benchmark_explore.py data/observations/floor-upright.jpg
```

Replay reports every labelled frame and fails if required categories are missing,
if any blocked frame is accepted or any clear example is blocked. Replay uses
the same person/pet and relative-depth veto implementation as live observation. The
saved-image benchmark is only a model benchmark, not live validation.

Profile `validation` requires reviewed evidence lists for `motion`, `camera`,
`watchdog`, `perception` and `latency`. The bridge checks evidence files exist and requires passing machine-generated
perception and latency reports bound to the exact configuration. It cannot
independently attest human measurements. A passed flag is never to be
invented. Mounting, focus, surface or model changes need a new reviewed profile.

```sh
.venv/bin/python scripts/calibrate.py check-profile data/calibration/explore-profile.json
.venv/bin/python scripts/chat_control.py explore-profile data/calibration/explore-profile.json --human-instruction 'ACTUAL HUMAN REVIEW'
```

## One supervised mission

Copy `config/mission.example.json`, fill the validated configuration, rooms,
directed doorways and actual human-confirmed starting clearance disk. A zero
radius grants no unseen side clearance. Bounds are at most 600 s and 10 m of
conservative cumulative wheel travel, including turns. Quotas reserve worst-case
travel over newly authorised lease time plus a complete fallback stopping
allowance per leg, and do not refund unused reservations.

```sh
.venv/bin/python scripts/chat_control.py session start --mode explore
.venv/bin/python scripts/mission.py propose data/mission.json
.venv/bin/python scripts/chat_control.py approve-mission MISSION_ID --human-instruction 'ACTUAL BOUNDED MISSION APPROVAL'
.venv/bin/python scripts/mission.py start MISSION_ID
.venv/bin/python scripts/mission.py candidates
.venv/bin/python scripts/mission.py select CANDIDATE_ID
.venv/bin/python scripts/mission.py status
```

Inspect the saved annotated candidate image before selecting. Only offered goals
can be chosen. They expire after 30 s, are consumed once, and are rechecked against
fresh geometry. The controller supports 10/25/50 cm candidates and <=15 degree
turns; each turn needs the entire swept area, including side/rear clearance.
The swept envelope includes the permitted 3 cm lateral drift and 5 degree
heading error; exceeding either stops the controller. Turning checks also
cover continued rotation during the verified command-loss fallback.
Approach and steering corrections run continuously within <=250 ms leases; no
LLM response is needed during a leg. Arrival requires tracked progress, not just
a timer. A one-second lack of progress or disagreement stops the leg.

At stopped checkpoints, use `mission.py annotate FILE` with a JSON `description`.
Optional `doorway` fields are `id`, `surface`, `uncertain`, `notes`, `distance_m`.
Distance is the assessed forward distance from chassis centre to the crossing
line at that checkpoint. Unknown geometry/surface stays uncertain and blocks
crossing. Select with `--doorway-id ID` only for an approved directed connection
whose crossing line plus chassis half-length and arrival margin fit in the leg.
Known doorway boundaries cannot be crossed by omitting the doorway ID. Observed
out-of-scope doorways can be recorded but cannot be selected for crossing.
Otherwise approach first, observe anew and annotate again. A reached crossing
records that forward edge; it never asserts the reverse route was demonstrated.

Use the camera and ask the human when room labels, surfaces, boundaries or
clearance are ambiguous. Geometry cannot independently recognise semantic room
boundaries. Revisit matches are suggestions requiring visual confirmation;
multiple plausible matches remain ambiguous. No old metric map is restored.

```sh
.venv/bin/python scripts/mission.py finish
.venv/bin/python scripts/mission.py report
```

Finish stops and pauses camera/model work. Reports are also written on faults.
STOP first cancels local renewals, even if the network Stop fails. STOP, faults,
pause, expiry, viewer loss and restart cancel remaining authority. Fresh approval
is needed to resume. Tracking loss requires a stationary session restart before
relocalisation. The Pi enforces lease/authority; semantic scope remains on Mac.

## Records and acceptance

Each mission saves events, raw and annotated checkpoint images, map/pose snapshots,
model/profile identity, observed doorways, directed traversals and `report.md`.
The latest ten distinct navigation frames are buffered and saved on failure.
No continuous recording is enabled. On restart, interrupted journals are marked
interrupted; saved knowledge never restores clearance or permission.

Use `config/acceptance.example.json` to collect actual evidence. Trial entries
are `{action, target, measured, stopped, direction_correct, evidence}`. Forward
units are metres, turn units degrees. Evidence paths are relative to the manifest.
Stop records require `measured_stop_m <= envelope_m`; supported-wheel firmware
fallback records require `measured_timeout_s <= verified_upper_timeout_s`.
Survey entries contain `{two_rooms, no_contact, recognised_finish, evidence}`.

```sh
.venv/bin/python scripts/validate_exploration.py acceptance data/acceptance.json --output records/explore-acceptance.json
.venv/bin/python -m unittest discover -s tests -v
```

Acceptance requires three trials per 10/25/50 cm target (error <= max(3 cm,20%)),
three left and right 15 degree trials (error <=5 degrees), recorded camera/network/
viewer/obstacle stops, verified firmware fallback, shutdown/disarmed restart,
and two successful two-room surveys. Software tests do not satisfy these gates.

Future hardware decision: `records/navigation-hardware-plan-20261013.md`.
