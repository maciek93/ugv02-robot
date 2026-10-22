# UGV02 software setup

Updated 12 October 2026 (Europe/London). Pi 5 + six-wheel UGV02/ESP32;
USB/UVC camera; no installed distance sensor, arm, lidar or ultrasonic sensor.

## Current result

**Paused at user request: `ugv-web` is stopped and autostart disabled.**
Camera capture and telemetry service are therefore off. Verified no app process
and no process holding video0. The camera remains electrically powered by USB.
For explicit future resumption: `sudo systemctl start ugv-web`; restore boot
autostart only when wanted with `sudo systemctl enable ugv-web`. The current
implementation captures continuously while running; making capture demand-driven
is a later change, not something configured during this shutdown.
Stop evidence: `records/camera-stop-verification.txt`. Shutdown left a failed
service status, now cleared: shutdown exceeded the 10-second timeout and systemd
killed the remaining processes. Graceful shutdown needs repair and verification
before further movement work. `records/camera-stop-service.txt` preserves the
log for later review. No further motion test was run during stopping.

- Live camera and fresh ESP32 telemetry work from the Mac.
- Seven supervised forward wheel pulses completed on a book support: requested
  0.08 m/s on both sides, 250 ms each. User confirmed four powered wheels turn
  (two per side), middle wheels stay passive, and all four stop after each pulse.
- Pi automatically sent zero speeds after each lease; journal timings for the
  captured pulses were approximately 252–270 ms. Feedback returned to zero.
- Controls are disarmed now. New sessions require explicit SSH arming, expire
  after at most 120 seconds, and reset to disarmed on service restart.
- Manual page is prepared and visually checked. Disarmed rejection, Stop,
and stale command rejection passed. Holding/releasing its direction buttons,
  focus-loss Stop and disconnect Stop still need supervised hardware tests.
- Floor driving waits for full charge and the user repositioning the robot.
  No unattended autonomous roaming is enabled. Actual boot/reboot recovery,
  operation under floor-driving load and sensor safeguards remain unverified.

## Verified inventory

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

## URLs and controls

Home-network camera/vendor page: http://ugvpi.local:5000/ . Its vendor movement,
raw JSON, PWM/ROS and peripheral routes remain blocked. No Jupyter or fallback
hotspot was installed. The original ESP32 firmware's independent Wi-Fi control
path has not been audited or disabled.

Mac manual page: http://127.0.0.1:5001/manual . This uses an authenticated SSH
tunnel; direct LAN access to the manual page/movement API is rejected. The tunnel
ends if its SSH connection closes. Setup on the Mac, from this project:

```sh
open -a Terminal connect-pi.command
# Enter any SSH password only in that Terminal, never in chat or records.
ssh -S .runtime/ssh-control -O forward -L 127.0.0.1:5001:127.0.0.1:5000 ugvpi@ugvpi.local
```

For a NEW supervised session, with the robot still supported and user watching,
run on the Pi (or through the shared SSH connection):

```sh
python3 ~/robot-records/arm_session.py --seconds 60
python3 ~/robot-records/arm_session.py --disarm
```

The first command arms; the second disarms and sends Stop. Arming alone does
not move wheels. Never arm for unsupported/unattended operation. Manual buttons
request 0.08 m/s, refresh every 100 ms while held, and release/STOP cancels the
command. Browser blur/visibility/pointer cancellation sends Stop. Socket
sequence numbers reject old commands after Stop; disconnect stops the owner.

Pi guard: requested wheel speeds limited to +/-0.1 m/s; single movement leases
0.05–0.35 s; watchdog checks about every 20 ms. New motion requires fresh
telemetry (<0.75 s), fresh camera and temporary arming. One controller owns an
active lease. Serial writes are direct, bounded by a 250 ms write timeout;
there is no queued motor backlog. Normal service shutdown and camera failure
attempt Stop. The ESP32's documented roughly 3-second fallback remains relevant
if the Pi crashes, is killed, loses power, or cannot write Stop. These are
software timing policies, not hard realtime guarantees.

Important measurement limit: encoder feedback peaks around 0.19 m/s appeared
for the 0.08 m/s pulses; some short pulses had zero left-side samples despite
the user seeing both sides turn. Feedback is instantaneous and appears coarse;
quantization/low-speed response and calibration need review before floor tests.
The command cap is not a verified physical speed ceiling. Do not adjust PID,
encoder parameters or firmware based on these short samples alone.

Separate observation: `GET /snapshot.jpg` (fresh JPEG or HTTP 503, no cache).
`GET /health` reports actual camera, feedback, arming and movement status.
Bench/agent groundwork: local-only `POST /api/drive` with
`{"left":0.08,"right":0.08,"duration":0.25,"owner":"operator-id"}`;
`POST /api/stop` with `{}`. API arming is intentionally absent. Further agent
work needs stale/replayed REST request protection and collision sensing; do not
use this groundwork for unattended roaming.

## Installation and reviewed changes

Vendor source: `waveshareteam/ugv_rpi`, commit
`3ae9f203f61bfaaf123ee376f78247d007880b0c`; reference ESP32 source
`waveshareteam/ugv_base_ros`, commit `2e7df97c331ff63e1481afb34feec52014a3632a`.
Reference source is not proof of the installed ESP32 firmware version.

Reviewed setup/autorun, requirements, app, camera and controller code. We skipped
whole-OS upgrades, audio-file overwrites, OAK rules, hotspot packages and the
large vendor dependency snapshot. `autorun.sh` would enable Jupyter and remove
its authentication; it was not run. We installed the needed packages instead.
Remote commands:

```sh
sudo -n apt-get update
sudo -n apt-get install -y python3-venv python3-opencv python3-serial python3-yaml python3-psutil python3-netifaces python3-pil python3-pygame espeak git
git clone --depth 1 https://github.com/waveshareteam/ugv_rpi.git ~/ugv_rpi
cd ~/ugv_rpi
python3 -m venv --system-site-packages ugv-env
ugv-env/bin/pip install Flask==3.1.2 Flask-SocketIO==5.5.1 simple-websocket==1.1.0 imutils==0.5.4 imageio==2.37.0 imageio-ffmpeg==0.6.0 pyttsx3==2.99
ugv-env/bin/pip check
sudo -n dtoverlay uart0-pi5
sudo -n udevadm settle
python3 ~/robot-records/probe_uart.py
python3 ~/robot-records/verify_uart_roundtrip.py
sudo -n python3 ~/robot-records/persist_uart.py
```

A speech library's default voice failed at startup; optional initialization now
lets the camera app run without an audio device. Unused CSI/OAK/MediaPipe/WebRTC
paths were disabled. No gimbal or arm startup command is sent. Camera faults
fail observations rather than showing placeholders; failures attempt Stop and
exit for systemd recovery. New manual controls share the tested motion guard.

To reproduce on a clean checkout of the recorded vendor commit, apply the
combined current patch `patches/camera-bounded.patch`; copy
`stationary_ctrl.py`, `telemetry_ctrl.py`, `bounded_ctrl.py`, `motion_guard.py`,
`camera_stream.py` from `deploy/` into `~/ugv_rpi`, and `manual.html` into its
`templates/` directory. Alternatively run the four adaptation scripts in order:
`adapt_vendor.py`, `adapt_telemetry.py`, `adapt_bounded.py`, `adapt_manual.py`,
each with the vendor checkout path. Do not also apply the patch. Earlier patches
are historical stage snapshots. Transfer scripts to `~/robot-records`.

Boot config: backed up to `~/robot-records/config-before-uart.txt`, then appended
`dtoverlay=uart0-pi5` under `[all]` in `/boot/firmware/config.txt`. Runtime UART
works without reboot; boot verification is pending. Debug serial/Bluetooth stay
unchanged. If reverting, stop the app and remove that added block, or restore
its backup after checking for later config changes. Do not remove a live UART
overlay while a process has the device open.

## Service, logs and recovery

Reviewed `deploy/ugv-web.service` installed at
`/etc/systemd/system/ugv-web.service`. Runs as ugvpi with video/dialout groups;
private `/run/ugv-web` arming state is removed on restart; restart-on-failure,
5-second retry, journal logs. Enabled on boot and manual restarts verified;
a real reboot/crash-recovery test is pending. Five rapid failures hit start limit.
The vendor Flask development server remains in use for bring-up; production
server hardening is pending. No home-router port forwarding should be added.

```sh
sudo -n install -m 644 ~/robot-records/ugv-web.service /etc/systemd/system/ugv-web.service
sudo -n systemd-analyze verify /etc/systemd/system/ugv-web.service
sudo -n systemctl daemon-reload
sudo -n systemctl enable --now ugv-web
sudo systemctl status ugv-web --no-pager
sudo journalctl -u ugv-web -b -n 100 --no-pager
sudo journalctl -u ugv-web -f
sudo systemctl stop ugv-web
sudo systemctl restart ugv-web
sudo systemctl reset-failed ugv-web
sudo systemctl start ugv-web
sudo systemctl disable --now ugv-web
curl http://127.0.0.1:5000/health
```

If control disconnects, the lease stops it; close/reopen the SSH session/tunnel.
For camera failure, check USB enumeration and the stable capture path, then
reset/start the service. Chassis switch remains the physical emergency fallback.

## Test evidence and next steps

Background audit requested by user: only custom service added was `ugv-web`,
currently inactive (PID 0) and disabled. No process holds video0 or ttyAMA0.
No robot Python process, Jupyter, hostapd/dnsmasq, robot cron job or robot web
listener remains. User/root crontabs are absent. Pi network listeners are SSH
22, Avahi/mDNS (for .local names), and CUPS 631 on loopback. Existing desktop,
printing, audio and Bluetooth services came from the OS image and were not
added by this setup. They are candidates for later headless cleanup, not
removed during this read-only audit. Mac port 5001 has no listening tunnel;
the robot control browser session has no tabs. A temporary shared SSH master
remains on the Mac, configured to expire after 10 minutes of idle time.
Optional speech dependencies were installed for upstream import compatibility,
despite no speaker being installed; they do not run a service. Continuous idle
camera capture and the shutdown timeout are implementation problems to fix before
resuming the app. Audit files: `records/background-audit-pi.txt` and
`records/background-audit-extra.txt`.

- `records/pi-inventory.txt`, `camera-independent.txt`, `camera-independent.jpg`.
- Installation: `install.txt`, `python-install.txt`, resolved `python-freeze.txt`.
- UART: `uart-probe-115200.txt`, `uart-roundtrip.txt`, config backup.
- Camera/telemetry: `stationary-tests.txt`, `telemetry-tests.txt`. One initial
  stream test was interrupted by my deliberate restart, then rerun successfully.
- Guard: `motion-guard-tests.txt` (8 meaningful fake-transport tests, no hardware
  movement); `bench-disarmed-tests.txt` (disarmed/range/duration/bypass rejection);
  direct LAN movement API returned HTTP 403.
- Real pulses: `bench-pulse-01.txt` through `bench-pulse-07.txt`, corresponding
  service logs. User confirmed four wheels turn and stop after each pulse.
- Manual page: `manual-disarmed-tests.txt`; Mac screenshot `manual-ui.jpg`.
  Browser Stop while stationary worked; interrupting active motion with that
  button, held controls, focus loss and actual network-loss tests are pending.

Next: supervised manual-page bench tests; Stop during motion; disconnect/focus
loss; verify startup on actual reboot. Then floor tests when batteries are
charged and user safely repositions. Keep floor speed conservative until the
encoder/physical-speed behavior is understood.

VL53L4CD: still disconnected. Before attaching Ozzmaker's female 2x3 adapter,
verify mechanical access, extension continuity, orientation and existing bus
use. Intended Pi pins: 1=3.3 V, 3=SDA/GPIO2, 5=SCL/GPIO3, 6=GND; 2/4 carry 5 V.
Adafruit breakout is compatible with 3.3 V logic; do not assume the ESP32's I2C
connector is the Pi bus. Enable I2C after correct wiring, detect 0x29, validate
readings/stale/invalid data, then add a fail-closed forward safety input using
`forward_check`. Limited range/FOV cannot protect sides, rear or drop-offs;
wider sensing is a later extension. Current sensor hook needs retained movement
direction before it can enforce forward-only policy correctly.

RoArm-M2: no arm/mount/power hardware installed; leave unconfigured. Do not add
USB-C PSU while chassis GPIO power remains attached without verified isolation.
UPS first activation using the supplied 12.6 V charger is documented; the current
robot already powers up. No additional power-source changes were made.

## Sources

- https://www.waveshare.com/wiki/UGV02
- https://github.com/waveshareteam/ugv_rpi
- https://github.com/waveshareteam/ugv_base_ros
- https://www.raspberrypi.com/documentation/computers/configuration.html
- https://learn.adafruit.com/adafruit-vl53l4cd-time-of-flight-distance-sensor/
- https://thepihut.com/products/5mp-ov5640-auto-focus-usb-camera-board
- https://thepihut.com/products/qwiic-connector-and-cable-for-raspberry-pi
- https://www.waveshare.com/product/psu-12.6v2a-5.5-2.1-eu.htm
