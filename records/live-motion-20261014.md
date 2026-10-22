# Live floor motion, 14 October 2026

The robot used the Pi guardian, the USB camera, and 250 ms maximum motor leases. The human was beside it with STOP visible and had confirmed a clear prepared floor area. The early trials requested 0.05 m/s; later manual trials used 0.10 m/s or direct PWM. The ESP32 receives negative raw motor commands because the Pi motor-polarity configuration is `[-1, -1]`; its `odl` and `odr` fields retain raw signs.

| Trial | Command | Raw odometry change | Independent physical observation | Camera evidence |
| --- | --- | --- | --- | --- |
| First corrected-polarity trial | Forward, 1 s | -5/-5 | Human: moved forward roughly 5 cm | `data/observations/live-20261014-before.jpg`, `live-20261014-after-1s.jpg` |
| Second forward trial | Forward, 1 s | -7/-3 | Human: too short to judge | `data/experiments/20261014-091132-25a0ed/` |
| Right-labelled pulse | Turn, 0.25 s | -2/0 | Physical direction not confirmed | `data/observations/live-20261014-before-turn.jpg`, `live-20261014-after-turn.jpg` |
| Left-labelled pulse | Turn, 0.25 s | -1/-1 | Physical direction not confirmed | `data/observations/live-20261014-after-left.jpg` |
| Longer forward trial | Forward, 4 s | -28/-21 | Human: about 10–20 cm, veered left; contact result unspecified | `data/experiments/20261014-092101-2b9a87/` |
| Requested long run, before manual mode | Forward, 27 s requested; perception stopped it after about 0.8 s | See experiment record | Human: about 5 cm; did not get the requested continuous movement | `data/experiments/20261014-093133-ae8b58/` |
| Manual speed-PID run | Forward, 27 s at requested 0.05 m/s | -153/-168 | Human: series of pulses, veered right; physical distance not estimated | `data/experiments/20261014-093613-864a67/` |
| Manual speed-PID run, fewer setpoint writes | Forward, 10 s at requested 0.10 m/s | -124/-108 | Human: still surging, veered left, about 1 m | `data/experiments/20261014-094024-499b27/` |
| Manual direct-PWM run | Forward, 5 s at PWM 50/255 | -85/-84 | Human measured **82 cm with a tape**, smooth movement, slight left veer under 2 cm | `data/experiments/20261014-094616-86aad8/` |

The four-second trial stopped at its fixed deadline. Its recorded before/after images show a large change toward the cabinets. The raw odometry values are encoder-derived wheel travel, not a measured chassis displacement; they also disagree with the rough 10–20 cm physical estimate.

There is a **left/right contradiction**: with conventional differential-drive geometry and correctly identified physical sides, a left wheel travelling farther than the right should veer right, whereas the human observed a left veer. The negative sign alone does not resolve this. The vendored source comments `A-left, B-right` and maps encoder A to `odl`, encoder B to `odr` (`vendor/ugv_base_ros/ROS_Driver/movtion_module.h`), but the running firmware and physical wiring have not been independently verified against that source. Plausible causes include swapped side labels, wheel slip, inaccurate encoder scale or a combination. Do not use these counts for steering correction, yaw or exploration until the side mapping is measured. Forward direction has only one direct human confirmation; turn directions and straight tracking remain unverified.

The 10-second speed-PID run sent only ten identical `T=1` speed commands, about one per second, with no intermediate Stop. Re-sending the setpoint less often did not resolve the surging. The firmware's default heartbeat interval in the vendored source is 3 seconds, so a several-second command needs periodic refresh unless that setting is changed and verified. Telemetry during this run showed zero speed in 42 of 145 left and 50 of 145 right samples, with frequent jumps to about 0.19 m/s. This supports PID hunting as a cause, but does not alone prove the physical motion profile.

The direct-PWM trial used the firmware's `T=11` command, which bypasses the speed PID. It sent five identical nonzero commands about one second apart and one explicit stop at the five-second deadline. Telemetry had zero speed in only 3 of 71 left and 7 of 71 right samples; nonzero samples were near 0.19 m/s. The encoder estimate was ~85 cm versus **82 cm physically measured with a tape** (about 4% high over this one trial). The human observed smooth movement and less than 2 cm left drift. This is one measured straight trial, not a repeatability claim. Battery telemetry went from 10.17 V before to 9.94 V after; charging state may affect later comparisons.

Once smooth forward motion is established, a sustained bounded turn can identify actual yaw and encoder side mapping. Keep the guardian and STOP active. No checkerboard is needed for this side-mapping test.

## Manual turning results

The first `pwm_left` trial at logical -50/+50 for 1.6 s produced only about 5° of **physical right** turn (human estimate), with raw odometry +2/-2 cm. A 30 s repeat turned a further ~85° right; the human saw only the physical left wheels moving, and raw odometry changed +6/-57 cm. A second 30 s run at the same setting turned beyond 180° total, approximately 200° by human estimate, again with only physical left wheels moving. The robot was then manually repositioned against a floor-panel line, so subsequent tests do not share the earlier heading or route origin.

At logical -80/+50 for 5 s, only the physical **right** wheels reversed; raw odometry changed +61/-2 cm. The human measured roughly 85° of rotation using the panel line. This establishes that the telemetry channels are physically swapped: `odl` followed the right wheels, `odr` the left wheels in these tests. It also shows that PWM 50 can stall one side under turning load. The raw sign-to-physical-direction relationship during turns remains provisional.

At equal and opposing logical PWM -80/+80 for 5 s, both physical sides moved in opposite directions; raw odometry changed +60/-59 cm. For a fresh full-turn test, the human aligned the robot with the panel line. A 9.7 s turn at the same command moved both sides and finished about 45° short of 360° (approximately 315°); raw odometry changed +118/-116 cm. A 1.4 s continuation ended about 5–10° **past** 360° by the human's line estimate. The cumulative 11.1 s / approximately 365–370° is one measured turn, suggesting roughly 33°/s at PWM 80 under those conditions. The physical result is approximate, and the battery was near 9.5 V after the longer run. Evidence: `data/experiments/20261014-095013-6d5e75/`, `20261014-095131-ec4b76/`, `20261014-095256-6e8580/`, `20261014-095634-4f75eb/`, `20261014-095743-24fc12/`, `20261014-100044-0c34a0/`, and `20261014-100145-5d8234/`.

The human then manually realigned the robot with the panel line and requested 180°. Equal opposing PWM 80 for **5.5 s** produced a human-confirmed **180° right turn**, with raw odometry +66/-62 cm. This second balanced-turn result is consistent with the full-turn timing, though endpoint accuracy and repeatability across battery states and surfaces remain unmeasured. Evidence: `data/experiments/20261014-100354-a67275/`.

The tested turn is a **physical right turn** despite the initial `pwm_left` label in software. Do not use the old left/right labels for navigation until they are corrected and verified in code. A turn about one stationary side also translates the chassis; it is not equivalent to the balanced spin.

## Supervised left-turn repeat, 22 October 2026

The human measured the chassis as **25 cm long and 23 cm wide**. The LD06 centre is laterally centred and **4–5 cm behind the front edge**, placing it approximately **7.5–8.5 cm forward of the chassis centre**. This is a measured mounting position, not a measured mounting yaw.

Two continuous `pwm_spin_left` trials each ran for 5.5 seconds at equal opposing PWM 80. Software stopped cleanly after both. In the first trial, raw `odl`/`odr` changed by +34/−51; the human did not observe its endpoint. In the second, they changed by +52/−56, and the human observed a smooth **approximately 180° physical left turn with both wheel sides moving**. The second result confirms the opposite turn direction and suggests about 33°/s under these conditions, but it does not establish repeatability or accurate odometry. Evidence: `data/experiments/20261022-120612-87f7c8/` and `data/experiments/20261022-120648-e6db40/`.

Battery telemetry fell to **8.76 V** during the second trial and recovered to roughly 9.0 V at rest. Further motor calibration was paused pending a charge; speed and turn rate may change with battery voltage. The LiDAR yaw, stopping distance and ESP32 command-loss response are still unmeasured.
