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
- [📁 Src](./src/) — All source code (Raspberry Pi Python + ESP32 MicroPython)
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

Diego is responsible for the ESP32 firmware, PID control, and sensor integration. He has competed in the FIRST Robotics Competition in both the United States and Mexico, achieved 1st place at WRO 2025 Mexicali, and alongside Sergio Amid represented Mexico at the WRO Italy 2024 Open World Championship.

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
- **Computer vision** (OpenCV on Raspberry Pi) for traffic sign color detection and wall following
- **LIDAR** (RPLidar A1M8) for 360° distance mapping and wall-following in the open challenge
- **Ultrasonic sensors** (URM37 V5) for close-range obstacle avoidance and parking alignment
- **PID control** on the ESP32 for precise servo steering and motor speed regulation
- **UART serial communication** between the Raspberry Pi (high-level decisions) and ESP32 (real-time actuation)

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
| Weight | ~450 g |

> **Note for judges:** Exact dimensions measured with calipers are documented in the engineering logbook. Robot must be under 300 mm × 200 mm per WRO vehicle regulations.

| Robot evolution |
|----------------|
| ![Robot evolution across design versions](v-photos/evolution.jpg) |

We went through three major physical revisions. Version 1 used an off-the-shelf 4WD chassis with the electronics mounted loosely on top — this caused vibration noise in the ultrasonic sensors and camera. Version 2 redesigned the top plate to rigidly mount all electronics and moved the camera to the front. Version 3 (current) added a dedicated LIDAR mount tower and reorganized cable routing to eliminate interference.

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

This ~6.6 cm/s base speed provides enough controllability for the PID loop while still completing three laps in a competitive time. The N20 at 12V produces ~1.5 kg·cm of stall torque, sufficient to move our ~450g robot including a safety margin for carpet-surface friction.

**Steering: DIYmall 11KG Mini All-Metal Digital Servo**

The front axle uses the DIYmall 11KG coreless digital servo for steering. We chose an 11 kg·cm servo (rather than a lighter 3–5 kg·cm servo) because our front wheel assembly includes a 4mm-to-3mm steering shaft that creates mechanical friction. The heavier servo ensures fast, accurate response to PID commands without positional lag. The servo is calibrated to 90° center before mounting using a calibration sketch on the ESP32.

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
| ESP32 | 3.3V | 500mA | Regulated on-board from 5V input |
| RPLidar A1M8 | 5V | 500mA | Separate 5V rail from buck converter |
| N20 DC Motor (×2) | 12V | ~300mA each (running) / ~1.5A stall | Via DRV8871 driver from battery direct |
| Steering Servo | 5V | ~800mA peak | Via 5V rail |
| URM37 Ultrasonic (×2) | 5V | ~40mA each | Via 5V rail |
| Freenove Camera | 5V (CSI) | ~250mA | Powered through Pi CSI connector |

**Battery:** LiPo 3S 11.1V, 2200 mAh

We use a 3S LiPo because the N20 motors are rated for 12V and benefit from the full voltage range. A 2200 mAh capacity gives an estimated runtime of:

```
Total average current draw ≈ 1.5A (motors running) + 1A (Pi + sensors) = 2.5A
Estimated runtime = 2200 mAh ÷ 2500 mA ≈ 52 minutes
```

Three competition rounds are estimated at under 10 minutes total, giving more than 5× safety margin.

**Voltage regulation:**

A DC-DC buck converter steps the 11.1V battery down to a stable 5V 5A rail for the Raspberry Pi, LIDAR, servos, and sensors. The ESP32 is powered from the Pi's 5V GPIO pin. The motor driver (DRV8871) takes 12V directly from the battery to drive the N20 motors.

**Wiring diagram:**

> See [📁 Schemes](./schemes/) for the full wiring schematic (Fritzing + PDF export).

Key connections:
- Battery (+) → DRV8871 VM IN and Buck Converter IN
- Buck converter 5V OUT → Raspberry Pi USB-C, LIDAR 5V, Servo signal rail
- Raspberry Pi GPIO 14 (TX) / 15 (RX) → ESP32 GPIO 16 (RX) / 17 (TX) via 3.3V logic (no level shifter needed since ESP32 is 3.3V tolerant on UART)
- ESP32 GPIO 18 → DRV8871 IN1 (PWM speed/direction control)
- ESP32 GPIO 19 → DRV8871 IN2 (PWM speed/direction control)
- ESP32 GPIO 22 → Servo signal wire (PWM)

### Sensor selection and placement

**RPLidar A1M8 (1 unit) — top-center mount**

The LIDAR provides 360° distance scanning at up to 8m range and ~5.5Hz rotation speed. We mount it at the top center of the robot so it has unobstructed line-of-sight to all four track walls simultaneously. It is the primary sensor for the open challenge wall-following algorithm. We considered using only ultrasonic sensors for the open challenge, but found their narrow detection cone missed corner geometry; the LIDAR's full-circle scan eliminates this blind spot.

**URM37 V5 Ultrasonic Sensors (2 units) — front-left and front-right**

The ultrasonic sensors provide accurate distance readings between 2cm and 500cm. We use two sensors angled ~30° outward from center to give a wide forward detection cone. These are the primary close-range sensors for obstacle avoidance and for measuring parking zone alignment during the parallel parking maneuver. Ultrasonic sensors were preferred over IR sensors because the WRO track uses colored walls that can confuse infrared reflectance.

**Freenove 8MP Camera (1 unit) — front center, elevated 80mm**

The camera is mounted forward-facing at 80mm height, which gives a field of view that captures both the floor zone immediately ahead of the robot and traffic signs at their actual height on the track. Elevation was determined through testing: lower mounting caused the camera to see too much floor and miss vertical pillar colors; higher than 90mm clipped the near field.

**Sensor calibration:**

- LIDAR: Uses RPLidar SDK default factory calibration. We apply a ±5mm software offset correction measured against a reference wall.
- Ultrasonic: Calibrated with `distance = (pulse_width × 0.034) / 2` (cm). We verified accuracy against a ruler at 10cm, 50cm, and 100cm before each competition session.
- Camera: White balance set to auto; HSV color thresholds for red/green were calibrated under the competition venue lighting conditions the day before the event and stored as constants in the config file.

[▲ Menu](#contents)

---

## Software architecture

### System overview

Our software is split across two controllers, each handling what it does best:

- **Raspberry Pi 4B (Python 3):** High-level decision-making — image capture, color detection, lap counting, state management, and sending steering/speed commands over UART.
- **ESP32 (MicroPython):** Real-time control — receiving commands from the Pi, reading ultrasonic sensors, running the PID loop, and outputting PWM to the servo and motor driver.

This split was deliberate: the Raspberry Pi's Linux OS introduces scheduling jitter that makes real-time PWM unreliable. Offloading motor control to the ESP32 (a bare microcontroller) guarantees consistent 10ms PID loop timing.

### UART communication protocol

Commands are sent as plain ASCII strings terminated with `\n`. The Pi sends one command per control cycle (~100ms):

| Command format | Meaning |
|---------------|---------|
| `S:90\n` | Set servo to 90° (center) |
| `S:75\n` | Steer left (servo to 75°) |
| `S:105\n` | Steer right (servo to 105°) |
| `M:50\n` | Set motor PWM to 50% (forward) |
| `M:-30\n` | Set motor PWM to -30% (reverse) |
| `M:0\n` | Stop motors |
| `PARK\n` | Trigger parking sequence on ESP32 |

The ESP32 responds with sensor data: `US:L15,R22\n` (ultrasonic left 15cm, right 22cm).

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

- **STARTUP:** Pi boots, initializes camera and serial port, waits for button press.
- **CALIBRATE:** LIDAR takes a 1-second baseline scan to establish wall distances.
- **WALL_FOLLOW (Open Challenge):** LIDAR-based PID loop keeps the robot centered in the lane. Error = (left wall distance) − (right wall distance). Target error = 0 (centered).
- **OBSTACLE_AVOID (Obstacle Challenge):** Camera detects red/green pillars. If a red pillar is seen, the robot shifts its steering setpoint to hug the right wall. If green, it shifts to hug the left wall. The shift magnitude is proportional to the detected pillar's pixel position in the frame.
- **LAP_TRACKING:** The ESP32 counts how many times the start section is passed (detected by a specific LIDAR distance signature at the start wall). After 3 triggers, transitions to PARK_SEARCH.
- **PARK_SEARCH:** The robot drives slowly, scanning for the magenta parking zone markers using the camera.
- **PARKING:** Executes the parallel parking sequence (see Obstacle Management section).

### PID controller (ESP32)

The PID runs every 10ms on the ESP32:

```python
error = target_distance - current_distance   # from LIDAR or ultrasonic
integral += error * dt
derivative = (error - prev_error) / dt

output = Kp * error + Ki * integral + Kd * derivative
servo_angle = 90 + clamp(output, -30, 30)
```

**Tuned values (current):**
- Kp = 1.2 (proportional — primary correction)
- Ki = 0.05 (integral — eliminates steady-state drift on banked sections)
- Kd = 0.8 (derivative — dampens oscillation)

These values were determined through Ziegler–Nichols step response tuning: we set Ki=0 and Kd=0, increased Kp until sustained oscillation occurred (Ku ≈ 2.8, Tu ≈ 0.4s), then applied the Z-N formula for a PD controller.

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
├── raspberry_pi/
│   ├── main.py              # State machine and main loop
│   ├── vision.py            # OpenCV color detection functions
│   ├── lidar_utils.py       # RPLidar SDK wrapper and wall-distance extraction
│   ├── serial_comm.py       # UART send/receive to ESP32
│   ├── config.py            # All tunable constants (HSV ranges, PID values, etc.)
│   └── parking.py           # Parallel parking sequence logic
└── esp32/
    ├── main.py              # Boot and UART listener
    ├── pid_controller.py    # PID implementation
    ├── motor_control.py     # DRV8871 PWM control
    ├── servo_control.py     # Servo PWM output
    └── ultrasonic.py        # URM37 distance reading
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
3. **Entry:** Robot reverses and steers into the parking zone. The ultrasonic right-side sensor monitors the distance to the right wall to prevent contact.
4. **Straighten:** Once inside the zone, the robot straightens the servo to 90° and drives forward slightly to center itself.
5. **Confirm:** If both ultrasonics read < 25cm (walls on each side), the robot is confirmed parked and motors stop.

[▲ Menu](#contents)

---

## Systems thinking and engineering decisions

### Why Raspberry Pi 4 + ESP32 (not a single controller)?

We initially prototyped with only the Raspberry Pi controlling everything. The Linux scheduler caused inconsistent PWM timing — the servo jittered visibly and PID loop timing varied from 8ms to 40ms per cycle. Moving real-time control to the ESP32 (which runs a bare MicroPython REPL with deterministic timing) solved this completely. The Pi focuses only on image processing and high-level logic, which is its strength.

**Alternative considered:** Arduino Mega. Rejected because MicroPython on ESP32 is faster to iterate on than C++ for PID tuning, and the ESP32's dual-core architecture allows UART listening on Core 0 and PID execution on Core 1 simultaneously.

### Why LIDAR + ultrasonic + camera (not just camera)?

Early testing with camera-only navigation showed poor performance in corners, where the walls fall outside the camera's forward field of view. The LIDAR's 360° scan solves this — it sees all walls simultaneously and the wall-following error is always well-defined. The ultrasonic sensors complement the LIDAR for close-range work (< 30cm) where the LIDAR's minimum range (15cm) becomes a limitation.

**Alternative considered:** Single-camera line following. Rejected because the WRO 2026 track uses no floor markings — walls are the only navigational reference.

### Why DRV8871 motor driver (not L298N or DRV8833)?

The DRV8871 was chosen over the L298N we used in 2025 for three key reasons. First, efficiency: the DRV8871 uses N-channel MOSFETs and loses only ~5% of power as heat, compared to the L298N's ~30% loss — this means less thermal management and longer battery life. Second, built-in current regulation: the DRV8871 has integrated current sensing that limits peak current to 3.6A, protecting our N20 motors from stall damage without external circuitry. Third, size: the DRV8871 module is significantly smaller and lighter than the L298N, which matters for our weight budget.

We considered the DRV8833 (dual-channel, 1.5A/channel) but rejected it because our N20 motors draw up to 1.5A stall each — right at the DRV8833's limit, with no safety margin. The DRV8871's 3.6A headroom is much more comfortable. The trade-off is that the DRV8871 is single-channel, so we use one module per motor (two total), but the smaller board size makes this easy to accommodate on our chassis.

### Design iteration history

| Version | Key change | Reason / Result |
|---------|-----------|----------------|
| v1.0 | Off-shelf 4WD chassis, breadboard wiring | Testing baseline — high vibration, unstable sensors |
| v1.1 | Replaced breadboard with soldered protoboard | Eliminated loose-connection faults |
| v2.0 | Custom 3D-printed top plate, camera moved to front | Reduced vibration noise, improved camera FOV |
| v2.1 | Added LIDAR tower mount | LIDAR previously taped to chassis — now rigid and repeatable |
| v3.0 | (Current) Full cable management, dedicated power rails | Eliminated ground loop noise in ultrasonic readings |

### Risk analysis

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Camera loses detection under venue lighting | Medium | HSV thresholds recalibrated on-site; config.py is hot-swappable |
| UART communication dropout | Low | ESP32 has timeout fallback: if no command received in 500ms, enters slow straight-line mode |
| LiPo battery depleted mid-round | Low | Battery checked at >80% before each round; runtime >> round duration |
| Pillar knocked over by edge case | Medium | Slow default speed near obstacles (30% PWM) reduces impact force |
| Parking zone not detected | Medium | If not found after 20s of PARK_SEARCH, robot stops safely in start section |

[▲ Menu](#contents)

---

## Robot construction guide

### Component list

| Component | Quantity | Description | Link |
|-----------|----------|-------------|------|
| Slamtec RPLIDAR A1M8 360° 2D LIDAR Scanner | 1 | 360° LIDAR for wall following | [Amazon](https://www.amazon.com/dp/B07TJW5SXF) |
| URM37 V5 Ultrasonic | 2 | Close-range obstacle/parking distance | — |
| N20 DC Gear Motor 12V 30RPM Metal Gearbox | 2 | Rear-wheel drive | [Amazon](https://www.amazon.com/dp/B0DB26SYNP) |
| HobbyPark Brass 1.0 Beadlock Wheels & Tires for 1/18 TRX4M | 4 | Brass beadlock wheels + tires + foam inserts | [Amazon](https://www.amazon.com/dp/B0C3MNX4K7) |
| PATIKIL U-Joint Steering Shaft Coupler 4mm to 3mm | 1 | Universal joint connects servo to front axle | [Amazon](https://www.amazon.com/dp/B0FWJGLZ9V) |
| RC Front & Rear Axle Housing Set (TRX4M compatible) | 2 | Ackermann steering linkage | [Amazon](https://www.amazon.com/dp/B0CW2HFT57) |
| ESP32 WROOM-32 | 1 | Real-time motor/servo controller | [Amazon](https://www.amazon.com/dp/B08D5ZD528) |
| Raspberry Pi 4B (4GB) | 1 | Main compute / vision controller | [Amazon](https://a.co/d/084kiOZ5) |
| DIYmall 11KG Mini All-Metal Digital Servo (360° Coreless) | 1 | Front-wheel steering | [Amazon](https://www.amazon.com/dp/B0DX1XG18Y) |
| DRV8871 H-Bridge DC Motor Driver | 2 | PWM motor control, 3.6A peak, one per motor | [MercadoLibre](https://www.mercadolibre.com.mx/modulo-driver-drv8871-puente-h-control-motor-36a-65v-a-45v/up/MLMU3232504497) |
| Freenove 8MP Camera | 1 | Traffic sign color detection | [Amazon](https://www.amazon.com/dp/B0BZYPBS17) |
| LiPo 3S 11.1V 2200mAh | 1 | Main power source | — |
| DC-DC Buck Converter 5V 5A | 1 | Steps 11.1V down to 5V rail | [Amazon](https://www.amazon.com/dp/B0D7MR48LB) |
| 3×120 Dupont Jumper Cables 40cm (M-M, M-F, F-F) | 3 packs | Wiring between all modules | [MercadoLibre](https://articulo.mercadolibre.com.mx/MLM-3643032042-3pzs-120-jumper-cable-dupont-wire-40cm-cable-para-protoboard-_JM) |
| Velstron 1,112-piece M3/M4/M5/M6 Screws, Bolts & Nuts Kit | 1 | Chassis fasteners and assembly hardware | [MercadoLibre](https://www.mercadolibre.com.mx/kit-surtido-de-1112-piezas-de-tornillos-pernos-y-tuercas/up/MLMU582984840) |
| 5-Pack Rocker Switch ON/OFF Red 2-Pin 127V/10A | 1 | In-line with battery positive | [MercadoLibre](https://www.mercadolibre.com.mx/5-pzas-interruptor-onoff-rojo-2-pines-127v10a-rojo/p/MLM59606936) |

**Estimated total cost: ~$175 USD**

### Pre-installation checks

**Servo motor:** Connect to ESP32 and run calibration sketch to set 90° center before mounting. Physical center must match electrical center or the robot will always drift to one side.

**DC motor:** Apply 12V directly and confirm rotation direction matches expected forward. The red terminal dot indicates positive.

### Electrical wiring — servo motor

| Wire color | Function |
|------------|---------|
| Brown | GND |
| Red | VCC (5V) |
| Yellow | PWM signal from ESP32 |

**Power switch:** Wired in series with the battery positive lead. Always switch off when not in use to protect the LiPo.

### Safety notes

- Do not operate in humid environments
- Verify polarity before connecting battery
- Never short the LiPo terminals — use a fused connector
- Double-check all wiring before first power-on after any reassembly

### Board mounting

The 3D-printed top plate has mounting holes for:
- ESP32 development board
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
| 1 | Raspberry Pi 4B (4GB) | [Amazon](https://a.co/d/084kiOZ5) |
| 1 | ESP32 WROOM-32 | [Amazon](https://www.amazon.com/dp/B08D5ZD528) |
| 1 | Freenove 8MP Camera | [Amazon](https://www.amazon.com/dp/B0BZYPBS17) |
| 2 | DRV8871 H-Bridge DC Motor Driver | [MercadoLibre](https://www.mercadolibre.com.mx/modulo-driver-drv8871-puente-h-control-motor-36a-65v-a-45v/up/MLMU3232504497) |
| 2 | URM37 V5 Ultrasonic Sensor | — |
| 1 | Slamtec RPLIDAR A1M8 360° 2D LIDAR Scanner | [Amazon](https://www.amazon.com/dp/B07TJW5SXF) |
| 2 | N20 DC Gear Motor 12V 30RPM Metal Gearbox | [Amazon](https://www.amazon.com/dp/B0DB26SYNP) |
| 1 | DIYmall 11KG Mini All-Metal Digital Servo (360° Coreless) | [Amazon](https://www.amazon.com/dp/B0DX1XG18Y) |
| 4 | HobbyPark Brass 1.0 Beadlock Wheels & Tires for 1/18 TRX4M | [Amazon](https://www.amazon.com/dp/B0C3MNX4K7) |
| 1 | PATIKIL U-Joint Steering Shaft Coupler 4mm to 3mm | [Amazon](https://www.amazon.com/dp/B0FWJGLZ9V) |
| 2 | RC Front & Rear Axle Housing Set (TRX4M compatible) | [Amazon](https://www.amazon.com/dp/B0CW2HFT57) |
| 1 | LiPo 3S 11.1V 2200mAh | — |
| 1 | DC-DC Buck Converter 5V 5A | [Amazon](https://www.amazon.com/dp/B0D7MR48LB) |
| 3 packs | 3×120 Dupont Jumper Cables 40cm (M-M, M-F, F-F) | [MercadoLibre](https://articulo.mercadolibre.com.mx/MLM-3643032042-3pzs-120-jumper-cable-dupont-wire-40cm-cable-para-protoboard-_JM) |
| 1 | Velstron 1,112-piece M3/M4/M5/M6 Screws, Bolts & Nuts Kit | [MercadoLibre](https://www.mercadolibre.com.mx/kit-surtido-de-1112-piezas-de-tornillos-pernos-y-tuercas/up/MLMU582984840) |
| 1 | 5-Pack Rocker Switch ON/OFF Red 2-Pin 127V/10A | [MercadoLibre](https://www.mercadolibre.com.mx/5-pzas-interruptor-onoff-rojo-2-pines-127v10a-rojo/p/MLM59606936) |

**Estimated total cost: ~$175 USD**

### Component function summary

| Component | Function |
|-----------|---------|
| Raspberry Pi 4B | Image processing, state machine, high-level navigation logic |
| ESP32 | Real-time PID control, PWM output, ultrasonic sensor reading |
| Freenove Camera | Traffic sign color detection via OpenCV |
| Slamtec RPLIDAR A1M8 | 360° wall distance mapping for open challenge navigation |
| URM37 V5 (×2) | Close-range obstacle detection and parking alignment |
| DRV8871 Driver (×2) | Efficient H-bridge motor control for N20 drive motors (one per motor) |
| N20 DC Gear Motor (×2) | Rear-wheel drive, 30RPM at 12V |
| DIYmall 11KG Servo | Front-wheel Ackermann steering, coreless digital motor |
| HobbyPark Brass Wheels (×4) | High-grip 1.0" beadlock wheels for 1/18 TRX4M chassis |
| PATIKIL U-Joint Coupler | 4mm-to-3mm universal joint connecting servo shaft to front axle |
| RC Front & Rear Axle Set | Steering and drive axle housings |
| LiPo 3S 2200mAh | Main power (11.1V, ~50 min runtime) |
| Buck Converter 5V | Regulated 5V rail for Pi, LIDAR, sensors, servo |
| Dupont Jumper Cables | All inter-module wiring connections |
| M3/M4/M5/M6 Hardware Kit | Chassis assembly fasteners (screws, bolts, nuts, washers) |
| Rocker Switch ON/OFF Red 2-Pin | Power control switch in-line with battery positive |

### Software and libraries

| Platform | Language | Libraries | Role |
|----------|---------|----------|------|
| Raspberry Pi 4B | Python 3.11 | OpenCV 4.8 | Color detection, contour analysis |
| | | RPLidar SDK | LIDAR scan parsing |
| | | pyserial | UART communication with ESP32 |
| | | numpy, imutils | Array math, image utilities |
| ESP32 | MicroPython 1.22 | machine | GPIO, PWM, UART |
| | | time, ustruct | Timing, binary packing |
| | | Custom PID | Servo and distance control |

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

We are proud of how Team CICSA's robot evolved over this season. Starting from a vibration-prone breadboard prototype, we arrived at a stable, reproducible platform with clean cable routing, rigid sensor mounting, and a well-tuned PID controller. The split-controller architecture (Raspberry Pi for vision + ESP32 for real-time control) proved to be the right decision and we would recommend it to future teams.

**What worked well:**
- The LIDAR-based wall following is highly robust to lighting changes and requires no floor markings
- The HSV color detection is fast (~30ms per frame) and reliable when calibrated on-site
- The MicroPython PID loop on the ESP32 maintains consistent 10ms timing

**What we would improve with more time:**
- Implement a Kalman filter to fuse LIDAR and ultrasonic data for more accurate position estimation
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
7. Freenove ESP32 tutorials — https://github.com/Freenove/Freenove_Ultimate_Starter_Kit_for_ESP32
8. WRO Future Engineers Getting Started Guide — https://world-robot-olympiad-association.github.io/future-engineers-gs/

[▲ Menu](#contents)
