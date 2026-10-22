# Experiments and route evidence

Project: `/Users/maciejbargiel/Documents/Robot`. Commands below use its `.venv/bin/python`.
The installed skill is guidance; software and data remain in this project.

## Record physical context before floor trials

Create `data/active-configuration.json` after confirming fixed camera orientation
and the visible floor. Include a distinct `configuration_id`, `camera_pose_confirmed`,
room, floor surface, camera reference image, robot width/length when measured and
known blind spots. Changing mounting, orientation or surface needs a new context
and a short movement recheck. The experiment runner snapshots this configuration.

Floor trial examples:

```sh
.venv/bin/python scripts/chat_control.py session resume --mode calibration
.venv/bin/python scripts/experiment.py pulse_forward 0.1 --label floor-forward-100ms-1 --context clear-floor --approval 'ACTUAL HUMAN APPROVAL'
.venv/bin/python scripts/measurement.py data/experiments/TRIAL --travel-mm NUMBER --braking-mm NUMBER --notes 'ACTUAL HUMAN MEASUREMENT'
```

Run three 100 ms and three 250 ms forward trials, then three short tests each
turning direction after clearance confirmation. Measure final travel after Stop
with a ruler/tape. Derive yaw from before/after chassis centreline marks on paper.
Store angle using `--angle-deg`. For each measurement, retain units, surface,
uncertainty and any asymmetric or inconsistent motion.

`measurement.py` computes a draft conservative rate from the largest measured
travel/angle divided by motor-on time, with 25% margin. It scopes aggregation to
the same configuration and never activates tour. Reconcile encoder and physical
measurements before relying on centimetres; stop early if estimates are unreliable.

## Stationary obstacle validation

Collect matched RGB/depth samples after stopping, using a real human label and
measured distance. Examples: clear floor, box, chair leg, cable, reflective surface,
poor lighting. Reserve separately labelled validation examples before choosing
thresholds; do not relabel failures to make the chosen cue appear successful.

```sh
.venv/bin/python scripts/label_observation.py --label blocked --object box --distance-mm NUMBER --split calibration --notes 'HUMAN DISTANCE AND SCENE'
```

This helper verifies fresh RGB/depth hashes. The stored relative-depth JPEG is
an illustration, not metres. Read the actual cue and camera metadata in sample.json.
Compare clear/blocked examples across backgrounds and validation cases. Only
activate a veto that demonstrates discrimination within a documented envelope.
If the scalar cue fails, keep the result as evidence and improve the method or
restrict/clear the route; do not fabricate near limits or silently enable tour.

The Pi profile currently accepts upper_speed_m_s, upper_turn_deg_s, near_limit,
score_low, score_high and evidence. Build it only from validated measurements
and obstacle results, review its limits with the human, then use the trusted
`scripts/chat_control.py profile FILE --human-instruction 'ACTUAL HUMAN REVIEW'`.
Include the complete experiment references in evidence. The current implementation
does not independently attest that those human measurements are truthful.

## Build route knowledge through driving

Increase forward bounds through 5, 10, 25 and at most 50 cm after three successful
trials at each level. Use small, calibrated turns, fresh observations and chat
approvals. Recheck new surfaces. Narrow/uncertain doorway clearance needs a physical
measurement or clearing before driving through it.

Confirm room labels and direct doorway connections in chat. Scoped API
`POST /agent/room` accepts room and from_room (library, kitchen, living room,
corridor; from_room optional). It saves one selected reference image and a confirmed
edge; do not infer that the corridor connects every room. Use recorded images and
fresh observations to navigate; never blindly replay a motor sequence.

Aim to visit all four rooms and return along the learned route. Record each actual
transition, approximate travel, corrections, obstacles and return outcome.
`data/progress.json` must explicitly distinguish demonstrated routes, untested
connections, excluded conditions and next physical action. The suite is not a
completed household-driving deliverable while these remain unverified.
