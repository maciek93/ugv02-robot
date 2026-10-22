# Architecture Audit & Engineering Roadmap: LLM-Driven Autonomous Robot Navigation

**Target Platform**: Waveshare UGV02 tracked chassis, Raspberry Pi 5 (SBC), ESP32 chassis controller, USB/UVC monocular camera (640x480), Apple Silicon Mac (M5, 32 GB RAM).  
**Goal**: Enable high-level natural language driving commands via an LLM (*"Drive to the kitchen, then approach the blue armchair"*) on a single floor, despite monocular camera constraints and non-real-time LLM inference.

---

## 1. Executive Summary & Current State

The repository contains a safety-first mobile robot architecture:
* **Raspberry Pi 5 (`ugv-supervised`)**: Implements strict real-time safety, motor leases capped at $\le 250\text{ ms}$, a $100\text{ ms}$ renewal deadline, and an independent hardware watchdog (`guardian.py`).
* **Mac Host**: Runs heavy perception and mission coordination (`mac.py`, `vision.py`). It executes three models concurrently (SegFormer-B0 floor segmentation, CoreML Depth Anything V2 Small, and YOLO26n) in **58 ms warm median latency** (~16 Hz).
* **Test Suite**: 59 automated unit and regression tests pass across control policies, geometry checks, and watchdog boundaries.

### The Core Architectural Mismatch
The current system was built as a human-in-the-loop bench/calibration testbed. It breaks navigation down into discrete $10\text{ cm}$, $25\text{ cm}$, $50\text{ cm}$ straight segments and $15^\circ$ turns (`scripts/mission.py candidates` / `select`). 

Requiring an LLM or human operator to select candidates every $25\text{ cm}$ over a 10-meter hallway requires **30 to 40 sequential round-trips**. Because LLMs have inherent network and reasoning latency ($1\text{–}5\text{ s}$), asking an LLM to act as a micro-step path planner is slow, brittle, and incapable of closed-loop target approach.

To achieve your goal, the system must shift to **Hierarchical 3-Tier Autonomy**:
1. **High-Level Agent (LLM)**: Semantic reasoning & mission-level orchestration ($0.1\text{–}0.5\text{ Hz}$).
2. **Mid-Level Autonomy (Mac M5)**: Semantic scene understanding, topological graph navigation, continuous corridor path planning, and visual servoing ($5\text{–}15\text{ Hz}$).
3. **Low-Level Reflexes (Pi 5 + ESP32)**: Hard real-time lease enforcement, motor safety watchdog, and IMU/odometry feedback ($20\text{–}50\text{ Hz}$).

---

## 2. In-Depth Technical Audit

### A. Root-Cause Analysis: Physical Motion Stalling (Trials 1 & 2)

Records in `data/progress.json` and `data/experiments/20261013-150507-15f6de/trial.json` reveal an unresolved motion failure:
* **Trial 1**: Commanded forward for $2.0\text{ s}$ $\to$ moved backwards $\approx 125\text{ mm}$.
* **Trial 2**: Direction polarity updated to `[-1, -1]`. Motors commanded for $2.0\text{ s}$ $\to$ human reported **no physical movement**, yet encoders reported $-11 / -8\text{ cm}$.

#### Root Causes Identified in Code:
1. **Track Static Friction vs. 0.05 m/s Crawl Speed**:
   * The UGV02 is a heavy metal chassis with rubber tracks.
   * Speed is hardcoded to $0.05\text{ m/s}$ ($5\text{ cm/s}$) in `supervised/policy.py`.
   * In firmware (`vendor/ugv_base_ros/ROS_Driver/movtion_module.h:294`), motor PWM below `THRESHOLD_PWM = 23` (out of 255) is suppressed to 0:
     ```cpp
     outputA = pidA.Run(speedGetA);
     if (abs(outputA) < THRESHOLD_PWM) {
       outputA = 0;
     }
     ```
   * At $0.05\text{ m/s}$, the PID output oscillates around or below this deadband threshold. Static track friction prevents the robot from breaking away, while motor gearbox backlash and vibration cause the optical encoders to tick slightly.
2. **Quantized Telemetry (Integer Centimeters)**:
   * In `vendor/ugv_base_ros/ROS_Driver/ugv_advance.h:410`:
     ```cpp
     long int odl_cm = (en_odom_l * 100);
     jsonInfoHttp["odl"] = odl_cm;
     ```
   * At $5\text{ cm/s}$ for a $250\text{ ms}$ lease, expected travel is only $1.25\text{ cm}$.
   * Integer truncation ($1.25\text{ cm} \to 1\text{ cm}$) introduces massive quantization noise, frequently tripping `FloorTracker`'s strict visual vs. encoder threshold (`max_translation_disagreement_m = 0.03`).

### B. The Monocular Camera Challenge (No Depth Sensor)

1. **Pitch Sensitivity in Inverse Perspective Mapping (IPM)**:
   * `supervised/navigation.py` uses a static homography matrix (`image_to_ground`) to project floor pixels into metric world coordinates.
   * This assumes zero pitch and roll. In physical driving, motor torque and carpet transitions cause $1^\circ\text{–}3^\circ$ pitch variations.
   * At $2\text{ meters}$, a $2^\circ$ pitch tilt shifts perceived ground coordinates by $50\text{–}100\text{ cm}$.
   * `FloorTracker` (`navigation.py:68`) detects this scale shift and immediately trips `ValueError('Camera pitch, scale or floor changed')`, locking the mission.
2. **Relative Depth vs. Metric Range**:
   * CoreML `DepthAnythingV2Small` outputs affine-invariant relative disparity.
   * The codebase condenses this into a single scalar center-ROI `near_score`. While effective as a binary collision veto for massive obstacles, it cannot answer: *"Is the chair 1.5 meters or 0.6 meters away?"*

### C. Missing Semantic Stack
* **No Furniture Perception**: YOLO26n only outputs `person` and `pet` as safety vetoes. It has no bounding boxes for `chair`, `table`, `door`, or `couch`.
* **Fragile Place Recognition**: ORB feature matching in `revisit_candidates` is viewpoint-dependent and easily invalidated by lighting changes or approaching from a different heading.

---

## 3. Recommended Architectural Solutions

### A. Single-Camera 3D Perception Without a Depth Sensor

1. **Ground-Contact Homography (Metric Object Localization)**:
   * On a single flat floor, all grounded furniture (chair legs, tables, door frames) intersects the ground plane.
   * Compute SegFormer floor segmentation mask $\mathcal{M}_{floor}$.
   * Detect the target object bounding box $B = [u_{min}, v_{min}, u_{max}, v_{max}]$.
   * The lowest bounding box edge $[u_{center}, v_{max}]$ touching the segmented floor defines the physical ground contact point.
   * Projecting that contact point through the homography $H_{img \to ground}$ yields the **exact metric $(X, Y)$ ground position relative to the chassis**:
     $$\begin{bmatrix} X_{robot} \\ Y_{robot} \\ 1 \end{bmatrix} \sim H_{img \to ground} \begin{bmatrix} u_{center} \\ v_{max} \\ 1 \end{bmatrix}$$
   * No LiDAR, stereo camera, or ToF sensor is required.

2. **IMU-Compensated Dynamic Homography**:
   * Read the ESP32 IMU pitch angle $\theta = \operatorname{atan2}(a_x, \sqrt{a_y^2 + a_z^2})$ from incoming telemetry.
   * Adjust camera pitch dynamically in the IPM projection:
     $$R_{live} = R_{calibrated} \cdot R_x(\Delta \theta)$$
   * Eliminates false `floor changed` tracking faults during acceleration and braking.

3. **Image-Based Visual Servoing (IBVS) for Target Approach**:
   * Once the target object (e.g. `chair`) is detected, bypass metric map coordinates and engage visual servoing:
     * **Yaw Alignment**: $\omega_z = -k_p (u_{bbox\_center} - u_{image\_center})$
     * **Forward Approach**: $v_x = k_v (h_{target\_pixels} - h_{current\_pixels})$
   * The robot centers the chair in frame and drives forward until the bounding box height reaches a calibrated threshold (or contact point reaches $0.5\text{ m}$), then halts smoothly.

### B. Upgrading Perception Models on Mac M5
* **Open-Vocabulary Object Detection**: Replace YOLO26n with **YOLO-World (v2-S or v2-M)** running on Mac MPS.
  * Inference latency: $\approx 12\text{–}18\text{ ms}$.
  * Dynamically prompts for any text target: `["chair", "armchair", "doorway", "table", "person"]`.
* **Topological Place Recognition**: Replace ORB with **MobileCLIP** or **CosPlace** image embeddings ($512\text{-d}$ vector).
  * Checkpoints store semantic embeddings.
  * Dot-product similarity identifies current room location invariant to small perspective and illumination changes.

### C. Low-Level Firmware & Transport Fixes
1. **Overcome Static Friction**:
   * Raise nominal floor travel speed in `policy.py` from $0.05\text{ m/s}$ to $0.12\text{–}0.15\text{ m/s}$ ($12\text{–}15\text{ cm/s}$).
   * Add a $50\text{ ms}$ breakaway torque pulse ($0.20\text{ m/s}$) when starting from a dead stop.
2. **Millimeter Odometry**:
   * Update ESP32 `ugv_advance.h` to transmit raw accumulated encoder pulses `encoderPulsesA`, `encoderPulsesB` in telemetry instead of integer centimeters.

---

## 4. The LLM High-Level Interface

Expose **macro-skills** as structured tools to the LLM agent:

```python
tools = [
    {
        "name": "navigate_to_room",
        "description": "Navigate autonomously to a target room via the topological room graph.",
        "parameters": {
            "room_name": {"type": "string", "enum": ["kitchen", "living_room", "hallway", "office"]}
        }
    },
    {
        "name": "approach_object",
        "description": "Locate an object in the current room and drive to within 0.5m using visual servoing.",
        "parameters": {
            "object_name": {"type": "string", "description": "e.g., 'chair', 'blue armchair', 'dining table'"}
        }
    },
    {
        "name": "scan_environment",
        "description": "Perform a 360-degree in-place scan and return detected objects, doorways, and rooms."
    },
    {
        "name": "emergency_stop",
        "description": "Immediately cancel all active motor leases."
    }
]
```

### Execution Walkthrough
> **User**: *"Drive to the kitchen, then approach the chair."*

1. **Step 1 (LLM Reasoning)**: Calls `navigate_to_room(room_name="kitchen")`.
   * Mac looks up path: `current_room` $\to$ `corridor` $\to$ `doorway_kitchen`.
   * Mac navigation worker continuously tracks floor corridor and issues renewable $250\text{ ms}$ leases.
   * Arrives at kitchen doorway, verifies room landmark embedding, stops, and returns: `{"status": "arrived", "room": "kitchen"}`.
2. **Step 2 (LLM Reasoning)**: Calls `approach_object(object_name="chair")`.
   * Mac sets YOLO-World target to `"chair"`.
   * If not immediately visible, rotates in-place until detection confidence $> 0.6$.
   * Visual servoing locks onto bounding box, aligns heading, and drives forward until the chair base is $0.5\text{ m}$ ahead.
   * Stops motors and returns: `{"status": "reached", "target": "chair", "distance_m": 0.52}`.
3. **Step 3 (LLM Response to User)**: *"I have navigated to the kitchen and parked directly in front of the chair."*

---

## 5. Phased Implementation Roadmap

| Phase | Milestone | Core Files | Deliverable |
| :--- | :--- | :--- | :--- |
| **Phase 1** | **Physical Motion & Deadband Calibration** | `policy.py`, `guardian.py`, `transport.py` | Overcome track friction; verify reliable forward/turn floor travel at $0.12\text{ m/s}$. |
| **Phase 2** | **Dynamic IMU Homography & Contact Projection** | `navigation.py`, `explore_vision.py` | Real-time pitch compensation; metric $(X,Y)$ calculation from object bottom edges. |
| **Phase 3** | **Open-Vocabulary Furniture Detection** | `supplementary_vision.py`, `vision.py` | Deploy YOLO-World on Mac MPS for zero-shot chair, door, and table detection ($<20\text{ ms}$). |
| **Phase 4** | **Visual Servoing (Target Approach)** | New `supervised/servoing.py` | Closed-loop visual pursuit toward detected objects, halting safely at $0.5\text{ m}$. |
| **Phase 5** | **LLM Tool Interface & Room Graph** | New `scripts/llm_agent.py` | High-level agent tool runner executing multi-room navigation and object approach from natural language. |
