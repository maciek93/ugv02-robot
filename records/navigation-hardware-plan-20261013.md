# Future navigation hardware decision

Recorded 13 October 2026, Europe/London. **Planned, not purchased or integrated.**

**22 October update:** The OKdo LD06 kit, its USB adapter/mini-USB-to-USB-A lead,
and a forward VL53L5CX through an MCP2221A USB-to-I²C board are now purchased and
connected. Both adapters enumerate; the LiDAR streams packets and the VL53L5CX
returns repeated 8×8 frames after a driver fix. Mounting calibration and floor
navigation acceptance remain pending. The remainder below records the original
purchase decision as it stood on 13 October.

Preferred next purchase: **OKdo / LDRobot LD06 360-degree scanning lidar**, plus a
suitable USB serial/power adapter, matching connector cable and rigid mounting.
Keep the existing UVC camera for scene interpretation. Do not reuse the chassis
UART for the lidar. Check adapter logic levels, power budget, connector pinout,
clear scan plane and clearance around the Pi Active Cooler before assembly.

Candidate: https://www.ebay.co.uk/itm/395488981619
The seller describes an LD06 module with bracket. Search and opened-page prices
on 13 October differed, approximately **GBP 15-18**. Recheck the current UK price,
VAT/delivery, included sensor and adapter, and seller terms before purchasing.
No purchase was made as part of this decision.

Reason: measured surrounding geometry should support room mapping and
localisation more directly than a single forward range reading. Mapping and
navigation software are still required; the lidar does not drive the robot.

Limitations: one horizontal scan plane can miss cables and other low objects,
overhangs and descending stairs. Glass/reflective surfaces need explicit tests.
Use the camera and forthcoming VL53L4CD as complementary observations. None is
proof of complete chassis clearance on its own.

Alternatives considered:
- Pimoroni VL53L5CX: approximately GBP 19.50 including VAT; 8x8 measured depth,
  Qwiic/STEMMA-compatible board. Coarse forward coverage, up to 15 Hz at 8x8.
  https://thepihut.com/products/vl53l5cx-8x8-time-of-flight-tof-array-sensor-breakout
- Sipeed MaixSense A010: approximately GBP 39 delivered from a marketplace;
  experimental 100x100 ToF depth, specified 0.2-2.5 m. No RGB camera despite some
  listings saying RGBD. Driver/integration and scene performance need validation.
  https://en.wiki.sipeed.com/hardware/en/maixsense/index.html
- OAK-D Lite: manufacturer listing USD 269 before delivery/tax; substantially
  higher budget for stereo depth and its software ecosystem.
  https://checkout.luxonis.com/products/oak-d-lite-1

V1 software uses the current camera only. Future ranging observations must keep
sensor identity and timestamps; adding lidar must not silently activate a new
operating envelope or change existing mission approval semantics.
