<img width="842" height="468" alt="CICSA WRO 2026 Banner" src="https://github.com/user-attachments/assets/f008d91d-e2a0-4c5f-97a4-c56cfcc7fe91" />

# WRO 2026 Future Engineers — Team CICSA
### National Final · Mexico · Season 2026

This is the official repository of **Team CICSA** for the WRO 2026 Future Engineers National Final. It contains all engineering materials for our self-driving vehicle: source code, wiring diagrams, 3D models, photos, and a full engineering journal documenting our design process.

---

## Follow us!

| Facebook | YouTube | Instagram |
|---------------|---------------|----------------|
| [![Facebook](other/facebook.jpg)](https://www.facebook.com/share/1A1hz6zQSn/) | [![YouTube](other/youtube.png)](https://www.youtube.com/@CICSA_Academia) | [![Instagram](other/instagram.jpg)](https://share.google/ZzGMFZOurGsjhWN2D) |

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

**Gildardo Garcia** — Age 16 · Centro de Estudios Tecnológicos Industrial y de Servicios 128

Gildardo is responsible for the mechanical design and 3D-printed chassis components. He participated in the 2019 WRO Regional in Guadalajara in the primary category, where he obtained 2nd place. For 2026 he leads all physical construction and assembly decisions.

---

**Sergio Amid Hernández González** — Age 19 · Software Engineering, Kuepa University

Sergio Amid leads the computer vision and Raspberry Pi software. He has participated in numerous competitions: 2019 WRO Regional in Guadalajara (1st place, preparatory category), 2024 WRO National in Mexicali (1st place, preparatory category), and represented Mexico at the WRO World Championship in Italy in 2024.

---

**Diego Pereida Arochi** — Age 17 · Nogales High School

Diego is responsible for PID control, motor/servo driver integration, and LIDAR sensor tuning, all running natively on the Raspberry Pi. He has competed in the FIRST Robotics Competition in both the United States and Mexico, achieved 1st place at WRO 2025 Mexicali, and alongside Sergio Amid represented Mexico at the WRO Italy 2024 Open World Championship.

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
- **Computer vision** (OpenCV on Raspberry Pi) for traffic sign color detection and wall following, aided by an onboard **white LED illuminator** that keeps lighting on the pillars consistent regardless of the venue's ambient lighting
- **LIDAR** (RPLidar A1M8) for 360° distance mapping, wall-following in the open challenge, and close-range obstacle/parking-wall distance checks
- **PID control**, running natively in Python on the Raspberry Pi, for precise servo steering and motor speed regulation
- **Hardware-timed PWM** (via the `pigpio` daemon) driving the servo and motor driver directly from Raspberry Pi GPIO pins

For full game rules, visit the [WRO Official Site](https://wro-association.org/).

[▲ Menu](#contents)

---

## Robot overview

| Front | Back | Top |
|-------|------|-----|
| ![Front view](v-photos/front.jpg) | ![Back view](v-photos/back.jpg) | ![Top view](v-photos/top.jpg) |

| Bottom | Left | Right |
|--------|------|-------|
| ![Bottom view](v-photos/bottom.jpg) | ![Left view](v-photos/left.jpg) | ![Right view](v-photos/right.jpg) |

| Dimension | Value |
|-----------|-------|
| Length | ~220 mm |
| Width | ~150 mm |
| Height | ~130 mm |
| Weight | ~420 g |

> **Note for judges:** Exact dimensions measured with calipers are documented in the engineering logbook. Robot must be under 300 mm × 200 mm per WRO vehicle regulations.

| Robot evolution |
|----------------|
| ![Robot evolution across design versions](v-photos/evolution.jpg) |

We went through three major physical revisions. Version 1 used an off-the-shelf 4WD chassis with the electronics mounted loosely on top — this caused vibration noise in the camera. Version 2 redesigned the top plate to rigidly mount all electronics and moved the camera to the front. Version 3 (current) added a dedicated LIDAR mount tower, simplified the electronics stack to a single Raspberry Pi controller, and reorganized cable routing to eliminate interference.

[▲ Menu](#contents)

---

## Mobility management

### Chassis and drive system

Our robot uses a **rear-wheel drive, front-wheel steering** architecture — commonly called Ackermann steering — which mirrors a real car's geometry. This was a deliberate choice over differential drive because the WRO rules require steering-based vehicles with a single drive axle connection (one motor per side is prohibited).

**Drive motor: N20 DC 12V, 30 RPM**

We selected the N20 30 RPM variant after testing 100 RPM and 200 RPM versions of the same motor family. The higher-RPM motors caused the robot to overshoot turns and made PID tuning unstable at low speeds. At 30 RPM with our 42mm diameter wheels, the linear speed is approximately:

```
Wheel circumference = π × 42 mm ≈ 132 mm
Speed = 30 RPM × 132 mm/rev ÷ 60 s ≈ 66 mm/s
```

This ~6.6 cm/s base speed provides enough controllability for the PID loop while still completing three laps in a competitive time. The N20 at 12V produces ~1.5 kg·cm of stall torque, sufficient to move our ~420g robot including a safety margin for carpet-surface friction.

**Steering: DIYmall 11KG Mini All-Metal Digital Servo**

The front axle uses the DIYmall 11KG coreless digital servo for steering. We chose an 11 kg·cm servo (rather than a lighter 3–5 kg·cm servo) because our front wheel assembly includes a 4mm-to-3mm steering shaft that creates mechanical friction. The heavier servo ensures fast, accurate response to PID commands without positional lag. The servo is calibrated to 90° center before mounting using a calibration script run directly on the Raspberry Pi.

**Wheels: TRX4M 1/18 scale**

These wheels provide a balance between grip and rolling resistance. Their 42mm outer diameter was used in all speed calculations above.

**Turning radius estimation**

With our wheelbase of approximately 120mm and maximum servo deflection of ±30°, the minimum turning radius is approximately 200mm. This is well within the WRO track corner geometry.

### Steering and assembly photos

> See [📁 Models](./models/) for all 3D-printed STL files for the chassis plate, LIDAR tower, camera mount, and servo bracket.
> See [📁 V-photos](./v-photos/) for step-by-step assembly photos.

[▲ Menu](#contents)

---

## Power and sensor management

### Power system architecture

| Component | Operating Voltage | Peak Current | Notes |
|-----------|-----------------|-------------|-------|
| Raspberry Pi 4B | 5V | 2.5A | Via USB-C through buck converter |
| RPLidar A1M8 | 5V | 500mA | Separate 5V rail from buck converter |
| N20 DC Motor (×2) | 12V | ~300mA each (running) / ~1.5A stall | Via DRV8871 driver from battery direct |
| Steering Servo | 5V | ~800mA peak | Via 5V rail |
| Freenove Camera | 5V (CSI) | ~250mA | Powered through Pi CSI connector |
| White LED Illuminator | 5V | ~150mA | Mounted next to camera, powered from 5V rail |

**Battery:** LiPo 3S 11.1V, 2200 mAh

We use a 3S LiPo because the N20 motors are rated for 12V and benefit from the full voltage range. A 2200 mAh capacity gives an estimated runtime of:

```
Total average current draw ≈ 1.5A (motors running) + 1.05A (Pi + LIDAR + camera + LED) ≈ 2.55A
Estimated runtime = 2200 mAh ÷ 2550 mA ≈ 52 minutes
```

Three competition rounds are estimated at under 10 minutes total, giving more than 5× safety margin.

**Voltage regulation:**

A DC-DC buck converter steps the 11.1V battery down to a stable 5V 5A rail for the Raspberry Pi, LIDAR, servo, and LED illuminator. The motor driver (DRV8871) takes 12V directly from the battery to drive the N20 motors. With the ESP32 removed, the Raspberry Pi's own 3.3V GPIO pins now drive the DRV8871 logic inputs and the servo PWM line directly — no intermediate microcontroller or level shifting is required, since the DRV8871's logic inputs and standard hobby servos both accept 3.3V signal levels.

**Wiring diagram:**

> See [📁 Schemes](./schemes/) for the full wiring schematic (Fritzing + PDF export).

Key connections:
- Battery (+) → DRV8871 (×2) VM IN and Buck Converter IN
- Buck converter 5V OUT → Raspberry Pi USB-C, LIDAR 5V, Servo signal rail
- Raspberry Pi GPIO 18 (hardware PWM via `pigpio`) → DRV8871 #1 & #2 IN1, paralleled (both motors share the same rear axle and are driven identically)
- Raspberry Pi GPIO 23 → DRV8871 #1 & #2 IN2 (direction control)
- Raspberry Pi GPIO 12 (hardware PWM via `pigpio`) → Servo signal wire
- Buck converter 5V OUT → White LED illuminator (+), GND to common ground

### Sensor selection and placement

**RPLidar A1M8 (1 unit) — top-center mount**

The LIDAR provides 360° distance scanning at up to 8m range and ~5.5Hz rotation speed. We mount it at the top center of the robot so it has unobstructed line-of-sight to all four track walls simultaneously. It is the primary sensor for the open-challenge wall-following algorithm, and — with our sensor suite simplified to LIDAR + camera only — it also now handles close-range distance checks during parking, using the readings from the sector of the scan facing each side wall.

**Freenove 8MP Camera (1 unit) — front center, elevated 80mm**

The camera is mounted forward-facing at 80mm height, which gives a field of view that captures both the floor zone immediately ahead of the robot and traffic signs at their actual height on the track. Elevation was determined through testing: lower mounting caused the camera to see too much floor and miss vertical pillar colors; higher than 90mm clipped the near field. The camera's contour size (apparent pillar width in pixels) is also used as a rough proximity cue when a pillar is very close, complementing the LIDAR.

**White LED illuminator (1 unit) — mounted beside the camera**

Competition lighting varies a lot between venues, and our HSV thresholds are calibrated the day before the event — so any lighting shift on competition day was previously enough to shrink our color-detection accuracy. We added a small white LED illuminator right next to the camera lens to keep the pillars lit consistently regardless of the venue's overhead lighting. This stabilizes the red/green HSV thresholds and reduces the false-negative rate on pillar detection under dim or mixed lighting.

**Sensor calibration:**

- LIDAR: Uses RPLidar SDK default factory calibration. We apply a ±5mm software offset correction measured against a reference wall.
- Camera: White balance set to auto; HSV color thresholds for red/green were calibrated under the competition venue lighting conditions the day before the event and stored as constants in the config file.
- LED illuminator: Left on for the full run so lighting on the pillars stays constant; if a future venue is very bright, we can dim it or recalibrate the HSV thresholds with it on.

[▲ Menu](#contents)

---

## Software architecture

### System overview

All decision-making and control now runs in a single process on the **Raspberry Pi 4B (Python 3)**: image capture, color detection, LIDAR-based wall following, lap counting, state management, PID control, and direct PWM output to the servo and motor driver.

We previously split real-time control onto a dedicated ESP32 microcontroller to avoid Linux scheduling jitter. With the ESP32 removed, we mitigate that same jitter concern by using the `pigpio` daemon, which generates PWM pulses through the Raspberry Pi's DMA controller rather than in Python's own execution thread. This means the actual servo and motor pulses stay precisely timed even if the Python control loop itself is briefly delayed by the OS scheduler — the trade-off we accepted is a lower decision-update rate (~20ms per PID cycle, versus 10ms previously) in exchange for a much simpler wiring harness, one fewer point of failure, and no UART link to maintain.

### State machine

The robot operates as a finite state machine with the following states:

```
[STARTUP] → [CALIBRATE] → [OPEN_CHALLENGE or OBSTACLE_CHALLENGE]
                                    ↓
                              [LAP_TRACKING]
                             /              \
                    [WALL_FOLLOW]     [OBSTACLE_AVOID]
                             \              /
                              [LAP_COMPLETE]
                                    ↓
                          (after 3 laps) [PARK_SEARCH]
                                    ↓
                               [PARKING]
                                    ↓
                               [STOPPED]
```

- **STARTUP:** Pi boots, initializes camera, LIDAR, and GPIO/`pigpio` connection, waits for button press.
- **CALIBRATE:** LIDAR takes a 1-second baseline scan to establish wall distances.
- **WALL_FOLLOW (Open Challenge):** LIDAR-based PID loop keeps the robot centered in the lane. Error = (left wall distance) − (right wall distance). Target error = 0 (centered).
- **OBSTACLE_AVOID (Obstacle Challenge):** Camera detects red/green pillars. If a red pillar is seen, the robot shifts its steering setpoint to hug the right wall. If green, it shifts to hug the left wall. The shift magnitude is proportional to the detected pillar's pixel position in the frame.
- **LAP_TRACKING:** The main loop counts how many times the start section is passed (detected by a specific LIDAR distance signature at the start wall). After 3 triggers, transitions to PARK_SEARCH.
- **PARK_SEARCH:** The robot drives slowly, scanning for the magenta parking zone markers using the camera.
- **PARKING:** Executes the parallel parking sequence (see Obstacle Management section).

### PID controller (Raspberry Pi)

The PID loop runs on a dedicated Python thread at approximately 20ms intervals:

```python
error = target_distance - current_distance   # from LIDAR
integral += error * dt
derivative = (error - prev_error) / dt

output = Kp * error + Ki * integral + Kd * derivative
servo_angle = 90 + clamp(output, -30, 30)
pigpio_pi.set_servo_pulsewidth(SERVO_PIN, angle_to_pulsewidth(servo_angle))
```

**Tuned values (current):**
- Kp = 1.2 (proportional — primary correction)
- Ki = 0.05 (integral — eliminates steady-state drift on banked sections)
- Kd = 0.8 (derivative — dampens oscillation)

These values were originally determined through Ziegler–Nichols step response tuning on the previous ESP32 implementation (Ku ≈ 2.8, Tu ≈ 0.4s) and were re-validated after migrating the control loop to the Raspberry Pi, with only minor retuning needed thanks to `pigpio`'s stable pulse generation.

### Color detection (Raspberry Pi / OpenCV)

Traffic sign detection uses HSV color thresholding on each camera frame:

```python
# Convert BGR to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Red mask (wraps around 180°)
red_lower1 = np.array([0, 120, 70])
red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([170, 120, 70])
red_upper2 = np.array([180, 255, 255])

# Green mask
green_lower = np.array([40, 60, 40])
green_upper = np.array([80, 255, 255])
```

Contours are found with `cv2.findContours`. We discard contours with area < 500 pixels² to filter noise. The centroid X position of the largest matching contour determines how much to shift the steering setpoint.

### Code structure

```
src/
└── raspberry_pi/
    ├── main.py              # State machine and main loop
    ├── vision.py            # OpenCV color detection functions
    ├── lidar_utils.py       # RPLidar SDK wrapper and wall-distance extraction
    ├── pid_controller.py    # PID implementation (runs on Raspberry Pi)
    ├── motor_control.py     # DRV8871 PWM control via pigpio
    ├── servo_control.py     # Servo PWM output via pigpio
    ├── config.py            # All tunable constants (HSV ranges, PID values, etc.)
    └── parking.py           # Parallel parking sequence logic
```

[▲ Menu](#contents)

---

## Obstacle management

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
4. **Straighten:** Once inside the zone, the robot straightens the servo to 90° and drives forward slightly to center itself.
5. **Confirm:** If both LIDAR side readings show < 25cm (walls on each side), the robot is confirmed parked and motors stop.

[▲ Menu](#contents)

---

## Systems thinking and engineering decisions

### Why a single Raspberry Pi controller (not Raspberry Pi + ESP32)?

Our 2025 platform split real-time control onto a dedicated ESP32 microcontroller, because the Raspberry Pi's Linux scheduler introduced PWM jitter — the servo visibly stuttered and PID loop timing varied from 8ms to 40ms per cycle when everything ran on the Pi alone. For 2026 we revisited that decision to simplify the electronics stack, reduce weight, cut cost, and remove a UART link and an extra power rail as potential failure points.

The jitter problem itself hasn't gone away, but we now solve it differently: the `pigpio` daemon offloads actual pulse generation to the Raspberry Pi's DMA hardware, so the servo and motor PWM signals stay precisely timed regardless of what the Python interpreter is doing at that instant. What we do give up is decision-loop determinism — our PID now updates roughly every 20ms instead of a guaranteed 10ms — which we judged an acceptable trade-off given our top speed of ~6.6 cm/s and the track's corner geometry.

**Alternative considered:** Keeping the ESP32. Rejected for 2026 because the added wiring complexity and UART maintenance burden outweighed the timing benefit at our modest drive speed.

### Why LIDAR + camera (not ultrasonic sensors)?

Our previous design used two ultrasonic sensors for close-range obstacle avoidance and parking alignment, angled outward from center. In testing this season we found the LIDAR's minimum range (15cm) combined with its full 360° coverage — plus the camera's apparent-size cue for very close pillars — gave us close-range awareness that was accurate enough to drop the ultrasonic sensors entirely. This simplified our wiring, freed up current budget on the 5V rail, and removed two more components that could fail or drift out of calibration.

**Alternative considered:** Keeping ultrasonic sensors purely as a parking-alignment backup. Rejected because our practice runs showed the LIDAR's side-sector readings were consistently within a few millimeters of the ultrasonic readings they were replacing, making the redundancy unnecessary for our current track speed.

**Alternative considered (open challenge):** Single-camera line following. Rejected because the WRO 2026 track uses no floor markings — walls are the only navigational reference.

### Why DRV8871 motor driver (not L298N or DRV8833)?

The DRV8871 was chosen over the L298N we used in 2025 for three key reasons. First, efficiency: the DRV8871 uses N-channel MOSFETs and loses only ~5% of power as heat, compared to the L298N's ~30% loss — this means less thermal management and longer battery life. Second, built-in current regulation: the DRV8871 has integrated current sensing that limits peak current to 3.6A, protecting our N20 motors from stall damage without external circuitry. Third, size: the DRV8871 module is significantly smaller and lighter than the L298N, which matters for our weight budget.

We considered the DRV8833 (dual-channel, 1.5A/channel) but rejected it because our N20 motors draw up to 1.5A stall each — right at the DRV8833's limit, with no safety margin. The DRV8871's 3.6A headroom is much more comfortable. The trade-off is that the DRV8871 is single-channel, so we use one module per motor (two total), but the smaller board size makes this easy to accommodate on our chassis, and its logic inputs are driven directly from Raspberry Pi GPIO with no separate microcontroller needed.

### Design iteration history

| Version | Key change | Reason / Result |
|---------|-----------|----------------|
| v1.0 | Off-shelf 4WD chassis, breadboard wiring | Testing baseline — high vibration, unstable sensors |
| v1.1 | Replaced breadboard with soldered protoboard | Eliminated loose-connection faults |
| v2.0 | Custom 3D-printed top plate, camera moved to front | Reduced vibration noise, improved camera FOV |
| v2.1 | Added LIDAR tower mount | LIDAR previously taped to chassis — now rigid and repeatable |
| v3.0 | (Current) Removed ESP32 and ultrasonic sensors, consolidated all control onto the Raspberry Pi with `pigpio`-driven PWM, full cable management, dedicated power rails | Simplified wiring, cut cost and weight, eliminated UART maintenance and ground-loop noise from the removed ultrasonic sensors |

### Risk analysis

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Camera loses detection under venue lighting | Low | Onboard white LED illuminator keeps pillar lighting consistent; HSV thresholds recalibrated on-site; config.py is hot-swappable |
| PWM jitter from Python/OS scheduling | Low | `pigpio` generates pulses via DMA hardware, independent of Python loop timing |
| LiPo battery depleted mid-round | Low | Battery checked at >80% before each round; runtime >> round duration |
| Pillar knocked over by edge case | Medium | Slow default speed near obstacles (30% PWM) reduces impact force |
| Parking zone not detected | Medium | If not found after 20s of PARK_SEARCH, robot stops safely in start section |
| Close-range blind spot (LIDAR min. range 15cm) | Low | Camera apparent-size cue supplements LIDAR inside the blind zone; approach speed reduced near pillars and walls |

[▲ Menu](#contents)

---

## Robot construction guide

### Component list

| Component | Quantity | Description | Link |
|-----------|----------|-------------|------|
| Slamtec RPLIDAR A1M8 360° 2D LIDAR Scanner | 1 | 360° LIDAR for wall following and close-range checks | [Amazon](https://www.amazon.com/dp/B07TJW5SXF) |
| N20 DC Gear Motor 12V 30RPM Metal Gearbox | 2 | Rear-wheel drive | [Amazon](https://www.amazon.com/dp/B0DB26SYNP) |
| HobbyPark Brass 1.0 Beadlock Wheels & Tires for 1/18 TRX4M | 4 | Brass beadlock wheels + tires + foam inserts | [Amazon](https://www.amazon.com/dp/B0C3MNX4K7) |
| PATIKIL U-Joint Steering Shaft Coupler 4mm to 3mm | 1 | Universal joint connects servo to front axle | [Amazon](https://www.amazon.com/dp/B0FWJGLZ9V) |
| RC Front & Rear Axle Housing Set (TRX4M compatible) | 2 | Ackermann steering linkage | [Amazon](https://www.amazon.com/dp/B0CW2HFT57) |
| Raspberry Pi 4B (4GB) | 1 | Main compute / vision / control unit | [Amazon](https://a.co/d/084kiOZ5) |
| DIYmall 11KG Mini All-Metal Digital Servo (360° Coreless) | 1 | Front-wheel steering | [Amazon](https://www.amazon.com/dp/B0DX1XG18Y) |
| DRV8871 H-Bridge DC Motor Driver | 2 | PWM motor control, 3.6A peak, one per motor | [MercadoLibre](https://www.mercadolibre.com.mx/modulo-driver-drv8871-puente-h-control-motor-36a-65v-a-45v/up/MLMU3232504497) |
| Freenove 8MP Camera | 1 | Traffic sign color detection | [Amazon](https://www.amazon.com/dp/B0BZYPBS17) |
| White LED Illuminator Module (5V) | 1 | Stabilizes lighting on pillars for color detection | [Amazon](https://www.amazon.com/s?k=5v+led+illuminator+module) |
| LiPo 3S 11.1V 2200mAh | 1 | Main power source | — |
| DC-DC Buck Converter 5V 5A | 1 | Steps 11.1V down to 5V rail | [Amazon](https://www.amazon.com/dp/B0D7MR48LB) |
| 3×120 Dupont Jumper Cables 40cm (M-M, M-F, F-F) | 3 packs | Wiring between all modules | [MercadoLibre](https://articulo.mercadolibre.com.mx/MLM-3643032042-3pzs-120-jumper-cable-dupont-wire-40cm-cable-para-protoboard-_JM) |
| Velstron 1,112-piece M3/M4/M5/M6 Screws, Bolts & Nuts Kit | 1 | Chassis fasteners and assembly hardware | [MercadoLibre](https://www.mercadolibre.com.mx/kit-surtido-de-1112-piezas-de-tornillos-pernos-y-tuercas/up/MLMU582984840) |
| 5-Pack Rocker Switch ON/OFF Red 2-Pin 127V/10A | 1 | In-line with battery positive | [MercadoLibre](https://www.mercadolibre.com.mx/5-pzas-interruptor-onoff-rojo-2-pines-127v10a-rojo/p/MLM59606936) |

**Estimated total cost: ~$331 USD**

### Pre-installation checks

**Servo motor:** Connect to the Raspberry Pi (via `pigpio`) and run a calibration script to set 90° center before mounting. Physical center must match electrical center or the robot will always drift to one side.

**DC motor:** Apply 12V directly and confirm rotation direction matches expected forward. The red terminal dot indicates positive.

### Electrical wiring — servo motor

| Wire color | Function |
|------------|---------|
| Brown | GND |
| Red | VCC (5V) |
| Yellow | PWM signal from Raspberry Pi (`pigpio`) |

**Power switch:** Wired in series with the battery positive lead. Always switch off when not in use to protect the LiPo.

### Safety notes

- Do not operate in humid environments
- Verify polarity before connecting battery
- Never short the LiPo terminals — use a fused connector
- Double-check all wiring before first power-on after any reassembly

### Board mounting

The 3D-printed top plate has mounting holes for:
- Raspberry Pi 4B (standard 58mm hole spacing)
- Camera bracket (front-center, with LED illuminator mounted alongside)
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
| 1 | White LED Illuminator Module (5V) — ~$6 | [Amazon](https://www.amazon.com/s?k=5v+led+illuminator+module) |
| 2 | DRV8871 H-Bridge DC Motor Driver — ~$4 each | [MercadoLibre](https://www.mercadolibre.com.mx/modulo-driver-drv8871-puente-h-control-motor-36a-65v-a-45v/up/MLMU3232504497) |
| 1 | Slamtec RPLIDAR A1M8 360° 2D LIDAR — ~$99 | [Amazon](https://www.amazon.com/dp/B07TJW5SXF) |
| 2 | N20 DC Gear Motor 12V 30RPM — ~$8 each | [Amazon](https://www.amazon.com/dp/B0DB26SYNP) |
| 1 | DIYmall 11KG Mini All-Metal Digital Servo — ~$16 | [Amazon](https://www.amazon.com/dp/B0DX1XG18Y) |
| 4 | HobbyPark Brass Beadlock Wheels & Tires 1/18 TRX4M — ~$20/set | [Amazon](https://www.amazon.com/dp/B0C3MNX4K7) |
| 1 | PATIKIL U-Joint Steering Shaft Coupler 4mm→3mm — ~$8 | [Amazon](https://www.amazon.com/dp/B0FWJGLZ9V) |
| 2 | RC Front & Rear Axle Housing Set (TRX4M) — ~$12 | [Amazon](https://www.amazon.com/dp/B0CW2HFT57) |
| 1 | LiPo 3S 11.1V 2200mAh — ~$25 | — |
| 1 | DC-DC Buck Converter 5V 5A — ~$10 | [Amazon](https://www.amazon.com/dp/B0D7MR48LB) |
| 3 packs | 3×120 Dupont Jumper Cables 40cm — ~$6/pack | [MercadoLibre](https://articulo.mercadolibre.com.mx/MLM-3643032042-3pzs-120-jumper-cable-dupont-wire-40cm-cable-para-protoboard-_JM) |
| 1 | Velstron 1,112-piece M3/M4/M5/M6 Hardware Kit — ~$18 | [MercadoLibre](https://www.mercadolibre.com.mx/kit-surtido-de-1112-piezas-de-tornillos-pernos-y-tuercas/up/MLMU582984840) |
| 1 | 5-Pack Rocker Switch ON/OFF Red 2-Pin 127V/10A — ~$4 | [MercadoLibre](https://www.mercadolibre.com.mx/5-pzas-interruptor-onoff-rojo-2-pines-127v10a-rojo/p/MLM59606936) |

**Estimated total cost: ~$331 USD**

### Component function summary

| Component | Function |
|-----------|---------|
| Raspberry Pi 4B | Image processing, state machine, PID control, and direct PWM output — all navigation logic in one controller |
| Freenove Camera | Traffic sign color detection via OpenCV, and close-range proximity cue |
| White LED Illuminator | Keeps lighting on pillars consistent, stabilizing HSV color detection |
| Slamtec RPLIDAR A1M8 | 360° wall distance mapping for open-challenge navigation and close-range/parking distance checks |
| DRV8871 Driver (×2) | Efficient H-bridge motor control for N20 drive motors (one per motor), driven directly from Pi GPIO |
| N20 DC Gear Motor (×2) | Rear-wheel drive, 30RPM at 12V |
| DIYmall 11KG Servo | Front-wheel Ackermann steering, coreless digital motor, driven directly from Pi GPIO |
| HobbyPark Brass Wheels (×4) | High-grip 1.0" beadlock wheels for 1/18 TRX4M chassis |
| PATIKIL U-Joint Coupler | 4mm-to-3mm universal joint connecting servo shaft to front axle |
| RC Front & Rear Axle Set | Steering and drive axle housings |
| LiPo 3S 2200mAh | Main power (11.1V, ~55 min runtime) |
| Buck Converter 5V | Regulated 5V rail for Pi, LIDAR, servo |
| Dupont Jumper Cables | All inter-module wiring connections |
| M3/M4/M5/M6 Hardware Kit | Chassis assembly fasteners (screws, bolts, nuts, washers) |
| Rocker Switch ON/OFF Red 2-Pin | Power control switch in-line with battery positive |

### Software and libraries

| Platform | Language | Libraries | Role |
|----------|---------|----------|------|
| Raspberry Pi 4B | Python 3.11 | OpenCV 4.8 | Color detection, contour analysis |
| | | RPLidar SDK | LIDAR scan parsing |
| | | pigpio | Hardware-timed PWM output to servo and DRV8871 motor drivers |
| | | numpy, imutils | Array math, image utilities |
| | | Custom PID | Servo and distance control, runs natively on the Pi |

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

We are proud of how Team CICSA's robot evolved over this season. Starting from a vibration-prone breadboard prototype, we arrived at a stable, reproducible platform with clean cable routing, rigid sensor mounting, and a well-tuned PID controller. Consolidating everything onto a single Raspberry Pi — with `pigpio` handling hardware-timed PWM — let us simplify our wiring and cut both cost and weight versus our previous ESP32 + ultrasonic setup, and we would recommend this simplified architecture to future teams building at a similar drive speed.

**What worked well:**
- The LIDAR-based wall following is highly robust to lighting changes and requires no floor markings
- The HSV color detection is fast (~30ms per frame) and reliable when calibrated on-site
- `pigpio`'s DMA-based PWM kept servo and motor pulses stable even without a dedicated microcontroller

**What we would improve with more time:**
- Implement a Kalman filter to fuse LIDAR and camera-derived distance estimates for more accurate position estimation
- Add an IMU (MPU6050) to detect and correct for wheel slip during sharp corners
- Improve the parking algorithm to use pixel-level magenta zone detection rather than distance-based thresholding
- Add a web dashboard on the Pi for real-time parameter tuning over Wi-Fi during practice sessions

We thank Coach Sergio Iván for his support and guidance throughout the season, and CICSA Academy for providing the resources and space to build and test our robot.

[▲ Menu](#contents)

---

## References

1. WRO 2026 Future Engineers General Rules — https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf
2. WRO 2026 Documentation Rubric — https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Documentation-Rubric.pdf
3. RPLidar A1M8 SDK — https://github.com/Slamtec/rplidar_sdk
4. OpenCV HSV color detection — https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
5. Ziegler–Nichols PID tuning method — Ziegler, J.G. & Nichols, N.B. (1942). *Transactions of the ASME*, 64, 759–768.
6. Elecrow 4WD Car Installation Instructions — https://www.elecrow.com/download/4wd_CAR_Install_instructions.pdf
7. pigpio library documentation — https://abyz.me.uk/rpi/pigpio/
8. WRO Future Engineers Getting Started Guide — https://world-robot-olympiad-association.github.io/future-engineers-gs/

[▲ Menu](#contents)
