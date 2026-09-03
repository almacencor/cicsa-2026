<img width="842" height="468" alt="CICSA WRO 2026 Banner" src="https://github.com/user-attachments/assets/f008d91d-e2a0-4c5f-97a4-c56cfcc7fe91" />

# WRO 2026 Future Engineers — Team CICSA
### National Final · Mexico · Season 2026

This is the official repository of **Team CICSA** for the WRO 2026 Future Engineers National Final. It contains all engineering materials for our self-driving vehicle: source code, wiring diagrams, 3D models, photos, and a full engineering journal documenting our design process.

---

## Follow us!

| Facebook | YouTube | Instagram |
|---------------|---------------|----------------|
| [![Facebook](other/facebooklogo.png)](https://www.facebook.com/share/1A1hz6zQSn/) | [![YouTube](other/youtube.png)](https://www.youtube.com/@CICSA_Academia) | [![Instagram](other/instagram.jpg)](https://share.google/ZzGMFZOurGsjhWN2D) |

---

## Contents

**Folders**
- [📁 Models](./models/) — 3D printable parts (STL/F3D files)
- [📁 Other](./other/) — Engineering logbook and supplementary materials
- [📁 Schemes](./schemes/) — Wiring diagrams and electrical schematics
- [📁 Src](./src/) — All source code (Raspberry Pi Python — single-controller architecture)
- [📁 T-photos](./t-photos/) — Team photos (official + funny)
- [📁 V-photos](./v-photos/) — Vehicle photos (6 angles required)
- [📁 Video](./video/video.md) — Performance video links

**Index**
- [The team](#the-team)
- [The challenge](#the-challenge)
- [Robot overview](#robot-overview)
- [Mobility management](#mobility-management)
- [Power and sensor management](#power-and-sensor-management)
- [Software architecture](#software-architecture)
- [Obstacle management](#obstacle-management)
- [Systems thinking and engineering decisions](#systems-thinking-and-engineering-decisions)
- [Robot construction guide](#robot-construction-guide)
- [Engineering materials](#engineering-materials)
- [Performance videos](#performance-videos)
- [Digital engineering logbook](./other/README.md)
- [Final remarks and future work](#final-remarks-and-future-work)
- [References](#references)

---

## The team

**CICSA WRO 2026 — Future Engineers**

**Coach: Sergio Iván Hernández Ruiz**

Coach Sergio Iván provides the technical guidance and leadership required to keep Team CICSA on track. With extensive experience in robotics, he helps us navigate complex engineering challenges, refine our designs, and develop solutions that work in competitive environments. He has been director of the CICSA robotics academy since 2015 and has participated in WRO events since 2019.

---

**Gildardo Garcia** — Age 17 · Centro de Estudios Tecnológicos Industrial y de Servicios 128

Gildardo is responsible for the programming. He participated in the 2019 WRO Regional in Guadalajara in the primary category, where he obtained 2nd place. For the 2026 championship, he is responsible for programming and calibrating the sensors to ensure they function correctly.

---

**Sergio Amid Hernández González** — Age 20 · Software Engineering, Kuepa University

Sergio Amid leads the design and construction of the robot's 3D parts. He has participated in numerous competitions: 2019 WRO Regional in Guadalajara (1st place, preparatory category), 2024 WRO National in Mexicali (1st place, preparatory category), and represented Mexico at the WRO World Championship in Italy in 2024.

---

**Diego Pereida Arochi** — Age 18 · United States Air Force Avionics technician

Diego is responsible for PID control, motor/servo driver integration, and LIDAR sensor tuning, all running natively on the Raspberry Pi. He has competed in the FIRST Robotics Competition in both the United States and Mexico, achieved 1st place at WRO 2025 Mexicali, and alongside Sergio Amid represented Mexico at the WRO Italy 2024 Open World Championship. Currently in boot camp in San Antonio, Texas, and wasn't able to attend the national championship.

[📁 T-photos](./t-photos/)

[▲ Menu](#contents)

---

## The challenge

The WRO 2026 Future Engineers challenge requires teams to build a fully autonomous self-driving vehicle that competes in two rounds:

**Open Challenge:** The vehicle must complete three (3) laps on a track with randomly placed inner walls. Direction (clockwise or counter-clockwise) is revealed at competition time. No obstacles are present; the goal is clean navigation and lap time.

**Obstacle Challenge:** The vehicle must complete three (3) laps on the same track, this time with randomly placed red and green traffic sign pillars. The robot must:
- Pass to the **right** of a **red** pillar
- Pass to the **left** of a **green** pillar
- Not knock over any pillar
- After completing three laps, locate the magenta-bordered parking zone and execute a **parallel parking maneuver**

Scoring is based on: laps completed, obstacles respected, parking success, lap time, and documentation quality (engineering journal and this GitHub repository).

Our robot addresses these requirements using:
- **LIDAR** (RPLidar A1M8) for 360° distance mapping, sector-based wall following, and initial track-direction detection
- **Computer vision** (OpenCV on Raspberry Pi), used in the Obstacle Challenge program for traffic sign color detection and parking-zone location
- **PID control**, running natively in Python on the Raspberry Pi via `gpiozero`, for closed-loop steering during wall following
- **Hardware-timed PWM** (via the `pigpio` pin factory backing `gpiozero`) driving the servo and motor drivers from Raspberry Pi GPIO pins

For full game rules, visit the [WRO Official Site](https://wro-association.org/).

[▲ Menu](#contents)

---

## Robot overview

| Front | Back | Top |
|-------|------|-----|
| ![Front view](v-photos/robot_delante.jpg) | ![Back view](v-photos/robot_atras.jpg) | ![Top view](v-photos/robot_arriba.jpg) |

| Bottom | Left | Right |
|--------|------|-------|
| ![Bottom view](v-photos/robot_abajo.jpg) | ![Left view](v-photos/robot_izquierda.jpg) | ![Right view](v-photos/robot_derecha.jpg) |

| Dimension | Value |
|-----------|-------|
| Length | ~230 mm |
| Width | ~145 mm |
| Height | ~190 mm |
| Weight | ~1174 g |

> **Note for judges:** Exact dimensions measured with calipers are documented in the engineering logbook. Robot must be under 300 mm × 200 mm per WRO vehicle regulations.


[▲ Menu](#contents)

---

## Mobility management

### Chassis and drive system

Our chassis employs a **rear-wheel drive** (RWD) system using a 400-RPM N20 motor, capped at 50% power to optimize battery life and improve LiDAR readings during turns—as higher speeds caused the LiDAR to struggle, thereby increasing the margin of error. The motor connects to the wheels via a differential to enable tighter turns, while the front end features an Ackermann steering system.

**Drive motor: N20 DC 12V, 400 RPM**

We selected the **N20 400 RPM variant** after testing 500 RPM and 1000 RPM versions of the same motor family. The higher-RPM motors caused the robot to overshoot turns and made PID tuning unstable at low speeds. We limit the motor power to just **200 rpm** (50% power) so that the robot has better PID control when turning in the shortest possible time and with the 2.09in diameter wheels, the linear speed is approximately:

```
2.09 in x 2.54 = 5.3086
R=5.3086 / 2=2.654
C=3.1416 x 5.3086 = 16.667
16.667 cm/rev x 200rev/min = 3335.4 cm/min
3335.4 / 60 = 55.59cm/s
linear speed 55.6 cm/s
```

This ~55.6 cm/s base speed provides enough controllability for the PID loop while still completing three laps in a competitive time. The N20 at 12V produces ~1.5 kg·cm of stall torque, sufficient to move our 1.179 kg robot including a safety margin for carpet-surface friction. In code, drive speed is set as a duty-cycle fraction (`target_velocity`, currently 0.5) rather than a raw RPM value.

**Steering: DIYmall 11KG Mini All-Metal Digital Servo**

The front axle uses the DIYmall 11KG coreless digital servo for steering. We chose an 11 kg·cm servo (rather than a lighter 3–5 kg·cm servo) because our front wheel assembly includes a 4mm-to-3mm steering shaft that creates mechanical friction. The heavier servo ensures fast, accurate response to PID commands without positional lag.

In software, the servo is driven through `gpiozero`'s `AngularServo` class (backed by the `pigpio` pin factory for hardware-timed pulses), configured for a ±90° mechanical range with 0.5–2.5 ms pulse widths and a software center of 0°. Two calibration constants trim this in code:
- `STEERING_LIMIT = 40` — clamps every requested steering command to ±40°, well inside the mechanical range, to protect the linkage.
- `CENTER_OFFSET = -20` — a fixed trim added after clamping, to correct for the assembly's slight physical off-center bias, rather than a separate one-time calibration script.

**Wheels: TRX4M 1/18 scale**

These wheels provide a balance between grip and rolling resistance. Their 2.09IN outer diameter was used in all speed calculations above.

**Turning radius estimation**

With our wheelbase of approximately 120mm and a software steering limit of ±40°, the minimum turning radius is approximately 165mm. This is well within the WRO track corner geometry.

### Steering and assembly photos

> See [📁 Models](./models/) for all 3D-printed STL files for the chassis plate, LIDAR tower, camera mount, and servo bracket.
> See [📁 V-photos](./v-photos/) for step-by-step assembly photos.

[▲ Menu](#contents)

---

## Power and sensor management

### Power system architecture
|Component | Operating Voltage	| Approx. Typical Current	| Approx. High/Maximum Current	| Approx. Power |
|-----------|-----------------|-------------|-------|-------|
|Raspberry Pi 4B	| 5 V	| 0.7–1.2 A	| ~1.5–2.0 A	| 3.5–10 W |
|Freenove Camera	| 5 V	| ~0.1–0.3 A	| ~0.3–0.5 A	| 0.5–2.5 W |
|LiDAR A1M8	| 5 V	| ~0.12–0.18 A	| ~0.2 A	| 0.6–1 W |
|Mini Servo, 24.3 lb, 180°	| 5 V	| ~0.2–0.8 A	| ~1.5–2.0 A*	| 1–10 W |
|N20 Motor #1, 400 RPM	| 11.1 V via DRV8871	| ~0.2–0.8 A	| ~1.5 A or higher*	| ~2–17 W |

**Battery:** OVONIC 3S 11.1V, 2200 mAh

We use an 11.1V OVONIC 3S battery; although it nominally provides 12.6V, the voltage drops to 11.1V under load. It is capable of powering the N20 motor, as the motor primarily relies on current—which the 8871 driver supplies. A 2200 mAh capacity gives an estimated runtime of:1.11 hours if the motors are in a middle range of work and the processor is not demanding to much energy 

```
1.20+0.30+0.18+0.50=2.18
P5v = 5x2.18 = 10.9W
Pmotor = 11.1x0.80 = 8.88W
Ptotal = 10.9+8.88 = 19.78W
90% regulator efficiency = 21.98W
11.1V x 2.2Ah = 24.42Wh
Runtime = 24.42Wh/21.98W = 1.11H

```

Three competition rounds are estimated at under 10 minutes total, giving more than 5× safety margin.

**Voltage regulation:**

A DC-DC buck converter steps the 11.1V battery down to a stable 5V 5A rail for the Raspberry Pi, LIDAR, and servo. The motor drivers (DRV8871) take 12V directly from the battery to drive the N20 motors. The Raspberry Pi's own 3.3V GPIO pins drive the DRV8871 logic inputs and the servo PWM line directly — no intermediate microcontroller or level shifting is required, since the DRV8871's logic inputs and standard hobby servos both accept 3.3V signal levels.

**Wiring diagram:**

<img width="700" height="450" alt="image" src="t-photos/foto del circuito.png" />

> See [📁 Schemes](./schemes/) for the full wiring schematic (Fritzing + PDF export).

Actual GPIO assignments:
- Battery (+) → DRV8871 (1) VM IN and Buck Converter IN
- Buck converter 5V OUT → Raspberry Pi USB-C, LIDAR 5V, Servo signal rail
- Raspberry Pi **GPIO 23** → Rear-motor DRV8871 IN1 (forward)
- Raspberry Pi **GPIO 22** → Rear-motor DRV8871 IN2 (backward) — this is the channel actually driving the robot
- Raspberry Pi **GPIO 12** (hardware PWM via `pigpio`) → Servo signal wire

Each axle's driver has its own independent forward/backward pin pair, which is what allows the front motor to be enabled or disabled purely in software.

### Sensor selection and placement

**RPLidar A1M8 (1 unit) — top-center mount**

The LIDAR provides 360° distance scanning at up to 8m range. We mount it at the top center of the robot so it has unobstructed line-of-sight to the track walls. It is the primary sensor for both the open-challenge wall-following algorithm and the initial track-direction detection at start-up, and it also supports close-range distance checks in the Obstacle Challenge program (parking-wall proximity).

Rather than reading raw single-beam distances, the code partitions each scan into three angular sectors and runs a small DSP pipeline before using the data (see [Software architecture](#software-architecture) for details):
- Front sector: −5° to +5°
- Left sector: 255° to 285°
- Right sector: 75° to 105°

**Freenove 8MP Camera (1 unit) — front center, elevated 80mm**

Used by the Obstacle Challenge program (not present in `wro2026_open_e.py`). The camera is mounted forward-facing at 80mm height, which gives a field of view that captures both the floor zone immediately ahead of the robot and traffic signs at their actual height on the track. Elevation was determined through testing: lower mounting caused the camera to see too much floor and miss vertical pillar colors; higher than 90mm clipped the near field.

**Sensor calibration:**

- LIDAR: Uses RPLidar SDK default factory calibration. Points below 150mm or above 6000mm are discarded as out-of-range/self-mapping noise (see DSP pipeline below).
- Steering: mechanical/electrical zero-offset is trimmed with the `CENTER_OFFSET` constant in code rather than a separate calibration script.
- Camera (Obstacle Challenge program): white balance set to auto; HSV color thresholds for red/green calibrated under venue lighting the day before the event and stored as constants in the config file.
  

[▲ Menu](#contents)

---

## Software architecture

### System overview

All decision-making and control runs in a single process on the **Raspberry Pi 4B (Python 3)**, using the `gpiozero` library for motor/servo control with its **pigpio pin factory** (set via `GPIOZERO_PIN_FACTORY = 'pigpio'`) so PWM pulses are generated with hardware timing (via the Pi's DMA controller) instead of in Python's own execution thread. This means servo and motor pulses stay precisely timed even if the Python control loop itself is briefly delayed by the OS scheduler.

This repository currently documents two separate programs:
- **`wro2026_open_e.py` (Open Challenge)** — LIDAR-only: sector-based distance sensing, DSP filtering, a start-up direction-detection routine, and a velocity-form PID wall follower. **No camera, no OpenCV, no obstacle/parking logic exists in this file.**
- **Obstacle Challenge program** — extends the same LIDAR/PID/motor foundation with the camera-based color detection and parking sequence described in [Obstacle management](#obstacle-management).

### Open Challenge — quick overview

These diagrams are meant to be walked through verbally in under a minute.

**Flow — what the robot does, in order:**

```mermaid
flowchart TD
    A["Start"] --> B["Initialize<br/>(motors off, servo centered)"]
    B --> C["Find track direction<br/>(LIDAR spots the first opening)"]
    C --> D["Follow the wall<br/>(PID keeps it centered)"]
    D --> E["Turn the corner"]
    E --> F{"Finished all laps?"}
    F -- No --> D
    F -- Yes --> G["Slow final stretch, then stop"]
```

**States — what mode the robot is in:**

```mermaid
stateDiagram-v2
    [*] --> Starting
    Starting --> FindingDirection
    FindingDirection --> FollowingWall
    FollowingWall --> TurningCorner
    TurningCorner --> FollowingWall: more laps left
    TurningCorner --> Finishing: laps complete
    Finishing --> Stopped
    Stopped --> [*]
```

### Open Challenge — program flow

- **`initialize_system()`** — zeroes actuators (motors stopped, steering centered) before anything else runs.
- **LIDAR worker thread (`lidar_worker_thread`)** — a daemon thread that continuously reads scans from the RPLidar, splits each scan into front/left/right sectors by angle, runs the DSP pipeline below, and writes the resulting distances into shared global variables under a lock (`lidar_lock`) for the main thread to read.
- **`execute_start_sequence()`** — drives forward slowly while watching the left and right sectors. The first time one side reports a gap >1000mm while the front is ≤750mm, that's read as the track's first open corner; the robot executes a blind, timed 90° turn toward that opening and locks in which wall it will track (`control_direction`) for the rest of the run. Lap progress after that is tracked by counting a fixed number of turns rather than detecting a start/finish line.
- **`pid_wall_follower()`** — the closed-loop distance-holding routine, described in detail below.
- **`execute_90deg_left_turn()` / `execute_90deg_right_turn()`** — open-loop, timed turns (steer to ±40°, hold ~0.95s, recenter) used to negotiate each corner once the PID segment approaches it.
- **`terminate_system()`** — stops both motors and recenters steering; also runs on `KeyboardInterrupt` or any unhandled exception, so the robot fails safe.

### LIDAR signal processing (`filter_lidar_zone`)

Each angular sector's raw `(distance, angle)` points go through a small NumPy/SciPy pipeline before being used for control:

1. **Range filter** — discard points outside 150–6000mm (150mm excludes the robot's own chassis from self-mapping).
2. **Dropout interpolation** — missing/out-of-range samples are linearly interpolated (`np.interp`) from neighboring valid points rather than left as gaps.
3. **Median filter** — a wrapped median filter (window size 3) smooths the sector and rejects specular multi-path reflections.
4. **Step-discontinuity rejection** — the first-order difference between consecutive filtered points is checked against a 500mm gradient threshold; points that jump too fast between adjacent readings are dropped as likely noise.

The closest valid point in each sector is then taken as that sector's representative distance (front/left/right).

### PID controller (velocity-form, Raspberry Pi)

The actual PID implementation in `pid_wall_follower()` is a **velocity-form (incremental) PID**, not the textbook positional form:

```python
output = int(
    kp * (error - p_error)
    + (ki * error)
    + (kd * (error - (2 * p_error) + p_error_2))
    + p_output
)
output = min(max(output, -40), 40)
```

**Tuned values (current, in code):**
- `kp = 1.2` (proportional term, applied to the error delta)
- `ki = 0.0` (integral term currently disabled)
- `kd = 0.5` (derivative-of-error term, using a second-order error history)
- Sample time `T = 0.005s` — a ~200Hz control loop

The raw output is passed through `sanitize_control_signal()`, a rolling-median filter over the last 5 outputs: if a new value would jump more than 15° away from the recent median, it's replaced with that median instead, damping single-cycle spikes before they reach the servo. The sanitized value is what's actually sent to `set_steering_angle()`.

A safety early-exit ("CUELLO"/bottleneck check) is built into the long wall-follow segment: if the front sector reports ≤725mm after at least 250 cycles during the main 11-lap-loop call (not the short finishing call), the function exits early rather than continuing to drive toward a wall it's approaching too fast.

### Motor and steering actuation

- **`drive_forward_awd()` / `drive_backward_awd()` / `stop_awd()`** — the wiring supports both axles, but only the rear motor is engaged in normal operation; the front motor call is skipped unless `MOTOR_FRONT_FLAG` is `True`. Each axle has its own independent power-scaling constant (`FACTOR_FRONT`, `FACTOR_REAR`), currently both set to 1.00.
- **`set_steering_angle()`** — clamps the requested angle to `±STEERING_LIMIT` (40°), adds the `CENTER_OFFSET` trim (−20°), and writes the result to the `AngularServo`.

### Code structure

```
src/
└── raspberry_pi/
    ├── wro2026_open_e.py    # Open Challenge: state machine, LIDAR DSP, PID wall follower, motor/servo control
    └── (obstacle challenge program — vision/color detection, parking; separate file)
```

`wro2026_open_e.py` is a single self-contained script. Splitting it into separate modules (vision, LIDAR, PID, motor control) is on our list for a future cleanup pass.

[▲ Menu](#contents)

---

## Obstacle management

**Everything in this section describes the separate Obstacle Challenge program, which is not part of `wro2026_open_e.py`.** The Open Challenge script has no camera input and no color/parking logic.

### Traffic sign detection and response

When the camera detects a pillar:

1. A bounding box is drawn around the detected contour.
2. The centroid X position is normalized: `cx_norm = (cx - frame_width/2) / (frame_width/2)` → range [−1, 1].
3. If **red**: the robot's wall-follow setpoint shifts toward the **right wall** by `shift = 0.15 × (1 + cx_norm)` meters.
4. If **green**: the robot's wall-follow setpoint shifts toward the **left wall** by the same formula.
5. The shift decays back to center once the pillar is no longer visible.

**Edge case handling:** When two pillars of the same color appear simultaneously, we take the centroid of the larger contour. When red and green appear at the same time (rare), red takes priority (right-side passage is safer given typical track geometry).

### Parallel parking

The parking sequence is triggered after the third lap. Steps:

1. **PARK_SEARCH:** Robot drives at reduced speed (~30% PWM). Camera scans for two magenta rectangular markers using a dedicated HSV range for magenta.
2. **Alignment:** Once both markers are detected and separated by the expected pixel distance for the parking gap, the robot stops alongside the zone.
3. **Entry:** Robot reverses and steers into the parking zone. The LIDAR's side-facing readings are monitored to prevent contact with the right-hand wall.
4. **Straighten:** Once inside the zone, the robot straightens the servo to center and drives forward slightly to center itself.
5. **Confirm:** If both LIDAR side readings show < 25cm (walls on each side), the robot is confirmed parked and motors stop.

[▲ Menu](#contents)

---

## Systems thinking and engineering decisions

### Why gpiozero + pigpio (not a raw pigpio API or a separate ESP32)?

Our 2025 platform split real-time control onto a dedicated ESP32 microcontroller to avoid Linux scheduling jitter. For 2026 we removed the ESP32 to simplify the electronics stack, reduce weight, cut cost, and remove a UART link and an extra power rail as potential failure points.

We still needed to solve the jitter problem, so instead of bit-banging PWM in Python, our motor and servo control goes through `gpiozero`'s `Motor` and `AngularServo` classes, configured to use the `pigpio` pin factory. `pigpio` generates the actual pulses via the Raspberry Pi's DMA hardware, so timing stays precise regardless of what the Python interpreter is doing at that instant, while `gpiozero` gives us a simpler, higher-level API than calling the pigpio daemon's socket interface directly. The trade-off we accepted is a lower decision-update rate in our own control loop versus a dedicated microcontroller, which we judged acceptable given our top speed of ~6.6 cm/s and the track's corner geometry.

**Alternative considered:** Keeping the ESP32. Rejected for 2026 because the added wiring complexity and UART maintenance burden outweighed the timing benefit at our modest drive speed.

### Why a wired-but-disabled front motor instead of a true single-motor RWD build?

Our chassis and driver wiring currently support both axles (two independent DRV8871 channels, one per motor), with the front axle disabled purely by the `MOTOR_FRONT_FLAG` software flag. This grew out of testing both configurations during development and not yet having finalized which one we're locking in for finals.

**Alternative considered:** Physically wiring only the rear motor. This is the safer choice with respect to the single-drive-axle rule and is what we plan to do (or otherwise make unambiguous) before the next inspection — see the note in [Mobility management](#mobility-management).

### Why LIDAR + camera (not ultrasonic sensors)?

Our previous design used two ultrasonic sensors for close-range obstacle avoidance and parking alignment, angled outward from center. In testing this season we found the LIDAR's minimum range (15cm) combined with its full 360° coverage gave us close-range awareness that was accurate enough to drop the ultrasonic sensors entirely for the Open Challenge. This simplified our wiring, freed up current budget on the 5V rail, and removed two more components that could fail or drift out of calibration.

**Alternative considered:** Keeping ultrasonic sensors purely as a parking-alignment backup. Rejected because our practice runs showed the LIDAR's side-sector readings were consistently within a few millimeters of the ultrasonic readings they were replacing, making the redundancy unnecessary for our current track speed.

**Alternative considered (open challenge):** Single-camera line following. Rejected because the WRO 2026 track uses no floor markings — walls are the only navigational reference.

### Why DRV8871 motor driver (not L298N or DRV8833)?

The DRV8871 was chosen over the L298N we used in 2025 for three key reasons. First, efficiency: the DRV8871 uses N-channel MOSFETs and loses only ~5% of power as heat, compared to the L298N's ~30% loss — this means less thermal management and longer battery life. Second, built-in current regulation: the DRV8871 has integrated current sensing that limits peak current to 3.6A, protecting our N20 motors from stall damage without external circuitry. Third, size: the DRV8871 module is significantly smaller and lighter than the L298N, which matters for our weight budget.

We considered the DRV8833 (dual-channel, 1.5A/channel) but rejected it because our N20 motors draw up to 1.5A stall each — right at the DRV8833's limit, with no safety margin. The DRV8871's 3.6A headroom is much more comfortable. Each DRV8871 module drives one motor channel (front or rear), with independent forward/backward GPIO pin pairs, and its logic inputs are driven directly from Raspberry Pi GPIO with no separate microcontroller needed.

### Design iteration history

| Version | Key change | Reason / Result |
|---------|-----------|----------------|
| v1.0 | Off-shelf 4WD chassis, breadboard wiring | Testing baseline — high vibration, unstable sensors |
| v1.1 | Replaced breadboard with soldered protoboard | Eliminated loose-connection faults |
| v2.0 | Custom 3D-printed top plate, camera moved to front | Reduced vibration noise, improved camera FOV |
| v2.1 | Added LIDAR tower mount | LIDAR previously taped to chassis — now rigid and repeatable |
| v3.0 | (Current) Removed ESP32 and ultrasonic sensors, consolidated all control onto the Raspberry Pi via `gpiozero`/`pigpio`, full cable management, dedicated power rails | Simplified wiring, cut cost and weight, eliminated UART maintenance and ground-loop noise from the removed ultrasonic sensors |

### Risk analysis

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Camera loses detection under venue lighting (Obstacle Challenge) | Low | HSV thresholds recalibrated on-site the day before the event |
| PWM jitter from Python/OS scheduling | Low | `gpiozero`'s `pigpio` pin factory generates pulses via DMA hardware, independent of Python loop timing |
| LiPo battery depleted mid-round | Low | Battery checked at >80% before each round; runtime >> round duration |
| Front-motor wiring present but software-disabled could be flagged at inspection | Medium | Team evaluating physically removing/disconnecting the front motor before finals |
| Fixed-count (11-turn) lap loop rather than start-line detection could mis-navigate on an unexpected track layout | Medium | Practiced against the expected track geometry; a start-line/lap-crossing detector is on our future-work list |
| Close-range blind spot (LIDAR min. range 15cm) | Low | Approach speed reduced near walls; early-exit ("CUELLO") check stops the long wall-follow segment before a too-close approach |

[▲ Menu](#contents)

---

## Robot construction guide

### Component list

| Component | Quantity | Description | Link |
|-----------|----------|-------------|------|
| Slamtec RPLIDAR A1M8 360° 2D LIDAR Scanner | 1 | 360° LIDAR for wall following and close-range checks | [Amazon](https://www.amazon.com/dp/B07TJW5SXF) |
| N20 DC Gear Motor 12V 400RPM Metal Gearbox | 1 | Rear drive | [Amazon](https://www.amazon.com/dp/B0DB26SYNP) |
| HobbyPark Brass 1.0 Beadlock Wheels & Tires for 1/18 TRX4M | 4 | Brass beadlock wheels + tires + foam inserts | [Amazon](https://www.amazon.com/dp/B0C3MNX4K7) |
| PATIKIL U-Joint Steering Shaft Coupler 4mm to 3mm | 1 | Universal joint connects servo to front axle | [Amazon](https://www.amazon.com/dp/B0FWJGLZ9V) |
| RC Front & Rear Axle Housing Set (TRX4M compatible) | 2 | Ackermann steering linkage | [Amazon](https://www.amazon.com/dp/B0CW2HFT57) |
| Raspberry Pi 4B (4GB) | 1 | Main compute / vision / control unit | [Amazon](https://a.co/d/084kiOZ5) |
| DIYmall 11KG Mini All-Metal Digital Servo (360° Coreless) | 1 | Front-wheel steering | [Amazon](https://www.amazon.com/dp/B0DX1XG18Y) |
| DRV8871 H-Bridge DC Motor Driver | 1 | PWM motor control, 3.6A peak, one per motor channel (front + rear) | [MercadoLibre](https://www.mercadolibre.com.mx/modulo-driver-drv8871-puente-h-control-motor-36a-65v-a-45v/up/MLMU3232504497) |
| Freenove 8MP Camera | 1 | Traffic sign color detection (Obstacle Challenge program) | [Amazon](https://www.amazon.com/dp/B0BZYPBS17) |
| OVONIC 3S 11.1V 2200mAh LiPo Battery | 1 | Main power source | [Amazon](https://www.amazon.com/dp/B0D8SZRGJT) |
| DC-DC Buck Converter 5V 5A | 1 | Steps 11.1V down to 5V rail | [Amazon](https://www.amazon.com/dp/B0D7MR48LB) |
| 3×120 Dupont Jumper Cables 40cm (M-M, M-F, F-F) | 3 packs | Wiring between all modules | [MercadoLibre](https://articulo.mercadolibre.com.mx/MLM-3643032042-3pzs-120-jumper-cable-dupont-wire-40cm-cable-para-protoboard-_JM) |
| Velstron 1,112-piece M3/M4/M5/M6 Screws, Bolts & Nuts Kit | 1 | Chassis fasteners and assembly hardware | [MercadoLibre](https://www.mercadolibre.com.mx/kit-surtido-de-1112-piezas-de-tornillos-pernos-y-tuercas/up/MLMU582984840) |
| 5-Pack Rocker Switch ON/OFF Red 2-Pin 127V/10A | 1 | In-line with battery positive | [MercadoLibre](https://www.mercadolibre.com.mx/5-pzas-interruptor-onoff-rojo-2-pines-127v10a-rojo/p/MLM59606936) |

**Estimated total cost: ~$325 USD**

### Pre-installation checks

**Servo motor:** Connect to the Raspberry Pi (via `gpiozero`/`pigpio`) and confirm the software `CENTER_OFFSET` trim before mounting. Physical center must match the trimmed electrical center or the robot will always drift to one side.

**DC motors:** Apply 12V directly and confirm rotation direction matches expected forward for both the rear (active) and front (currently disabled) channels. The red terminal dot indicates positive.

### Electrical wiring — servo motor

| Wire color | Function |
|------------|---------|
| Brown | GND |
| Red | VCC (5V) |
| Yellow | PWM signal from Raspberry Pi GPIO 12 (`gpiozero`/`pigpio`) |

### Electrical wiring — motor drivers (as implemented)

| GPIO pin | Function |
|----------|---------|
| 23 | Rear motor DRV8871 IN1 (forward) — active |
| 22 | Rear motor DRV8871 IN2 (backward) — active |
| 12 | Servo PWM signal |

**Power switch:** Wired in series with the battery positive lead. Always switch off when not in use to protect the LiPo.

### Safety notes

- Do not operate in humid environments
- Verify polarity before connecting battery
- Never short the LiPo terminals — use a fused connector
- Double-check all wiring before first power-on after any reassembly

### Board mounting

The 3D-printed top plate has mounting holes for:
- Raspberry Pi 4B (standard 58mm hole spacing)
- Camera bracket (front-center)
- LIDAR tower (center, elevated ~60mm above chassis)

For all 3D files: [📁 Models](./models/)

[▲ Menu](#contents)

---

## Engineering materials

This repository contains all engineering materials for Team CICSA's self-driving vehicle competing in WRO Future Engineers 2026.

| Quantity | Component | Link |
|----------|-----------|------|
| 1 | Raspberry Pi 4B (4GB) — ~$55 | [Amazon](https://a.co/d/084kiOZ5) |
| 1 | Freenove 8MP Camera — ~$14 | [Amazon](https://www.amazon.com/dp/B0BZYPBS17) |
| 1 | DRV8871 H-Bridge DC Motor Driver — ~$4 | [MercadoLibre](https://www.mercadolibre.com.mx/modulo-driver-drv8871-puente-h-control-motor-36a-65v-a-45v/up/MLMU3232504497) |
| 1 | Slamtec RPLIDAR A1M8 360° 2D LIDAR — ~$99 | [Amazon](https://www.amazon.com/dp/B07TJW5SXF) |
| 1 | N20 DC Gear Motor 12V 400RPM — ~$8 | [Amazon](https://www.amazon.com/dp/B0DB26SYNP) |
| 1 | DIYmall 11KG Mini All-Metal Digital Servo — ~$16 | [Amazon](https://www.amazon.com/dp/B0DX1XG18Y) |
| 4 | HobbyPark Brass Beadlock Wheels & Tires 1/18 TRX4M — ~$20/set | [Amazon](https://www.amazon.com/dp/B0C3MNX4K7) |
| 1 | PATIKIL U-Joint Steering Shaft Coupler 4mm→3mm — ~$8 | [Amazon](https://www.amazon.com/dp/B0FWJGLZ9V) |
| 2 | RC Front & Rear Axle Housing Set (TRX4M) — ~$12 | [Amazon](https://www.amazon.com/dp/B0CW2HFT57) |
| 1 | OVONIC 3S 11.1V 2200mAh LiPo Battery — ~$25 | [Amazon](https://www.amazon.com/dp/B0D8SZRGJT) |
| 1 | DC-DC Buck Converter 5V 5A — ~$10 | [Amazon](https://www.amazon.com/dp/B0D7MR48LB) |
| 3 packs | 3×120 Dupont Jumper Cables 40cm — ~$6/pack | [MercadoLibre](https://articulo.mercadolibre.com.mx/MLM-3643032042-3pzs-120-jumper-cable-dupont-wire-40cm-cable-para-protoboard-_JM) |
| 1 | Velstron 1,112-piece M3/M4/M5/M6 Hardware Kit — ~$18 | [MercadoLibre](https://www.mercadolibre.com.mx/kit-surtido-de-1112-piezas-de-tornillos-pernos-y-tuercas/up/MLMU582984840) |
| 1 | 5-Pack Rocker Switch ON/OFF Red 2-Pin 127V/10A — ~$4 | [MercadoLibre](https://www.mercadolibre.com.mx/5-pzas-interruptor-onoff-rojo-2-pines-127v10a-rojo/p/MLM59606936) |

**Estimated total cost: ~$325 USD**

### Component function summary

| Component | Function |
|-----------|---------|
| Raspberry Pi 4B | State machine, LIDAR processing, PID control, and PWM output (via `gpiozero`/`pigpio`) — all navigation logic in one controller |
| Freenove Camera | Traffic sign color detection via OpenCV (Obstacle Challenge program only) |
| Slamtec RPLIDAR A1M8 | 360° sector-based wall distance mapping for navigation and direction detection |
| DRV8871 Driver | Efficient H-bridge motor control, driven directly from Pi GPIO |
| N20 DC Gear Motor| Rear-wheel drive active |
| DIYmall 11KG Servo | Front-wheel Ackermann steering, driven via `gpiozero`'s `AngularServo` (pigpio-backed) |
| HobbyPark Brass Wheels (×4) | High-grip 1.0" beadlock wheels for 1/18 TRX4M chassis |
| PATIKIL U-Joint Coupler | 4mm-to-3mm universal joint connecting servo shaft to front axle |
| RC Front & Rear Axle Set | Steering and drive axle housings |
| OVONIC 3S 2200mAh LiPo | Main power (11.1V, ~55 min runtime) |
| Buck Converter 5V | Regulated 5V rail for Pi, LIDAR, servo |
| Dupont Jumper Cables | All inter-module wiring connections |
| M3/M4/M5/M6 Hardware Kit | Chassis assembly fasteners (screws, bolts, nuts, washers) |
| Rocker Switch ON/OFF Red 2-Pin | Power control switch in-line with battery positive |

### Software and libraries

| Platform | Language | Libraries | Role |
|----------|---------|----------|------|
| Raspberry Pi 4B | Python 3 | `rplidar` | LIDAR scan parsing |
| | | `gpiozero` (pigpio pin factory) | Hardware-timed motor and servo control |
| | | `numpy`, `scipy.ndimage` | LIDAR DSP: interpolation, median filtering, gradient checks |
| | | Custom velocity-form PID | Wall-following steering control |
| | | OpenCV | Color detection, contour analysis — Obstacle Challenge program only |

[▲ Menu](#contents)

---

## Performance videos

> Video links are documented in [📁 Video](./video/video.md).

- Open Challenge — practice run (3 laps complete)
- Obstacle Challenge — practice run (3 laps + parking)
- Robot assembly time-lapse

[▲ Menu](#contents)

---

## Final remarks and future work

We are proud of how Team CICSA's robot evolved over this season. Starting from a vibration-prone breadboard prototype, we arrived at a stable, reproducible platform with clean cable routing, rigid sensor mounting, and a well-tuned PID controller. Consolidating everything onto a single Raspberry Pi — with `gpiozero`'s pigpio-backed PWM handling hardware timing — let us simplify our wiring and cut both cost and weight versus our previous ESP32 + ultrasonic setup, and we would recommend this simplified architecture to future teams building at a similar drive speed.

**What worked well:**
- The LIDAR-based wall following is robust to lighting changes and requires no floor markings
- The DSP pipeline (interpolation + median filter + gradient rejection) meaningfully cleaned up noisy LIDAR sectors before they reached the PID loop
- `gpiozero`'s pigpio pin factory kept servo and motor pulses stable even without a dedicated microcontroller

**What we would improve with more time:**
- Replace the fixed 11-turn loop with actual start/finish-line detection so lap counting is robust to any track layout, not just the practiced one
- Finalize the drivetrain as a true single-motor build (physically, not just via software flag) to remove any ambiguity around the single-drive-axle rule
- Implement a Kalman filter to fuse LIDAR and camera-derived distance estimates for more accurate position estimation
- Add an IMU (MPU6050) to detect and correct for wheel slip during sharp corners
- Improve the parking algorithm to use pixel-level magenta zone detection rather than distance-based thresholding
- Add a web dashboard on the Pi for real-time parameter tuning over Wi-Fi during practice sessions

We thank Coach Sergio Iván for his support and guidance throughout the season, and CICSA Academy for providing the resources and space to build and test our robot.

[▲ Menu](#contents)

- This is a small circuit made in Proteus showing the wiring layout described above. We didn't have every exact part on hand for the simulation, so some components are represented with similar substitute parts under different labels.

<img width="842" height="468" alt="image" src="[https://github.com/almacencor/cicsa-2026/blob/7a72de3f1409d025c9b163d7906e4dc845d0a45a/t-photos/foto%20del%20circuito.png]" />

---

## References

1. WRO 2026 Future Engineers General Rules — https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf
2. WRO 2026 Documentation Rubric — https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Documentation-Rubric.pdf
3. RPLidar A1M8 SDK — https://github.com/Slamtec/rplidar_sdk
4. OpenCV HSV color detection — https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
5. Ziegler–Nichols PID tuning method — Ziegler, J.G. & Nichols, N.B. (1942). *Transactions of the ASME*, 64, 759–768.
6. Elecrow 4WD Car Installation Instructions — https://www.elecrow.com/download/4wd_CAR_Install_instructions.pdf
7. gpiozero documentation — https://gpiozero.readthedocs.io/
8. pigpio library documentation — https://abyz.me.uk/rpi/pigpio/
9. WRO Future Engineers Getting Started Guide — https://world-robot-olympiad-association.github.io/future-engineers-gs/

[▲ Menu](#contents)
