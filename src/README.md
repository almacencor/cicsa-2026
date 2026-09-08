Control software
====

This directory must contain code for control software which is used by the vehicle to participate in the competition and which was developed by the participants.

All artifacts required to resolve dependencies and build the project must be included in this directory as well.

# Main 26_05_26

## PID Wall-Following Robot

This repository contains the first version of a MicroPython program for a mobile robot that maintains a reference distance from a side wall.

The system uses an ultrasonic sensor to measure the distance on the left side. A PID controller processes this measurement and changes the steering servo angle while the robot moves forward.

## Project Status

This is the *first experimental version*. Its main purpose is to test:

- Continuous ultrasonic distance measurement.
- Steering servo control.
- Motor movement using PWM.
- Trajectory correction using a PID controller.

The PID constants and servo limits still require physical calibration on the robot.

## Hardware

- Microcontroller compatible with MicroPython.
- Side-mounted ultrasonic sensor.
- SG90 servo motor for steering.
- DC motor or motors.
- Motor driver compatible with PWM signals.
- Suitable power supply for the motors, servo, and microcontroller.

## Pin Connections

| Component | Signal | GPIO |
|---|---|---:|
| LED | Output | 2 |
| Left ultrasonic sensor | TRIG | 4 |
| Left ultrasonic sensor | ECHO | 15 |
| SG90 servo motor | PWM | 21 |
| Motor driver | RPWM | 23 |
| Motor driver | LPWM | 22 |

> *Important:* all modules must share a common ground. Verify that the ultrasonic sensor's ECHO voltage is safe for the microcontroller. Do not power the motor or servo directly from a GPIO pin.

## How It Works

1. The LED turns on briefly to indicate that the program has started.
2. A second thread continuously reads the left ultrasonic sensor.
3. The robot begins moving forward at a constant PWM value.
4. The controller compares the measured distance with the reference distance.
5. The PID controller processes the resulting error.
6. The PID output changes the steering servo angle.
7. The motor stops after the configured number of control iterations.

## PID Controller

The current reference distance is *15 cm*. The initial controller values are:

| Constant | Value |
|---|---:|
| Kp | 6 |
| Ki | 0.02 |
| Kd | 2.6 |

The calculated steering angle is limited between *45° and 105°, with **70°* used as the initial center position.

The main function call used in this version is:

python
pid(15, 600, 10000)


The arguments represent:

- 15: reference distance in centimeters.
- 600: number of control iterations.
- 10000: PWM value used for forward movement.

## Software Requirements

- MicroPython firmware installed on the microcontroller.
- Thonny, mpremote, or another compatible tool for uploading and running the program.
- No external libraries are required.

## Running the Program

1. Connect all components according to the pin table.
2. Save the main program as main.py on the microcontroller.
3. Place the robot on a safe surface with enough room to move.
4. Turn on the system and observe the measurements displayed in the console.
5. Adjust Kp, Ki, Kd, the center angle, and the speed according to the robot's physical response.

The console displays information similar to:

text
Distance: 15 PID Output: 0.0 Angle: 70


## Main Program Structure

- forward(value): controls forward movement using PWM.
- set_servo_angle(angle): converts an angle from 0° to 180° into a PWM signal for the servo.
- get_distance(echo, trig): obtains a measurement from the ultrasonic sensor.
- get_data(): continuously updates the distance in a second thread.
- pid(ref, condition, speed): executes the trajectory controller.

## Planned Improvements

- Correct and validate the ultrasonic trigger pulse.
- Handle invalid measurements or None values without stopping the program.
- Filter unstable sensor measurements.
- Calculate the PID controller using a real sampling interval.
- Add limits to the accumulated output to prevent saturation.
- Add an emergency stop.
- Organize the program into separate modules.
- Document the exact microcontroller and motor-driver models.

## Testing Safety

Perform the first tests with the wheels raised or at low speed. Keep the movement area clear and use an appropriate power supply to prevent resets or damage to the electronic components.

## License

This project is published for educational and experimental development purposes. A formal license may be added in a future version.

# wro2026_obstacle.py

## WRO 2026 Future Engineers - Obstacle Challenge

Autonomous vehicle developed for the *WRO 2026 Future Engineers Obstacle Challenge*. The system runs on a Raspberry Pi 4 and combines a 2D LiDAR, a camera, and an MPU6050 inertial sensor to follow walls, determine the driving direction, avoid colored traffic pillars, negotiate corners, recover from critical situations, and complete the required laps.
 
> Main program: wro2026_obstacle.py

## Project goals

The controller was designed around five priorities:

- Compliance with the WRO Future Engineers obstacle rules.
- Repeatable behavior with different battery levels.
- Sensor validation instead of acting on isolated measurements.
- A clear finite-state machine for testing and debugging.
- Recovery from temporary faults without immediately ending the run.

## Hardware

| Component | Purpose | Connection |
|---|---|---|
| Raspberry Pi 4 | Main computer and control coordinator | Central controller |
| RPLidar A1 | Front and lateral distances, wall geometry, open spaces, and corner detection | USB serial, normally /dev/ttyUSB0 |
| Freenove IMX219 camera | Red/green pillar detection and image position | Raspberry Pi CSI interface |
| MPU6050 | Angular velocity, accumulated heading, and 90-degree turn confirmation | I2C: SDA GPIO 2, SCL GPIO 3 |
| Ackermann steering servo | Front steering angle | GPIO 12 |
| Rear DC motor and driver | Forward and reverse movement | GPIO 23 and GPIO 22 |

The LiDAR convention used by the software is:

- 0°: front
- 90°: right
- 180°: rear
- 270°: left

Positive steering angles turn left; negative angles turn right.

## System architecture

The sensors run in independent background threads. Each thread publishes its newest measurement with a sequence number and timestamp. A single fixed-rate controller reads the latest snapshots at *40 Hz*, validates them, executes the active state, and sends commands to the steering servo and rear motor.

mermaid
flowchart TD
    CAM["Camera<br/>pillar color, x, bottom, area"]
    LIDAR["RPLidar A1<br/>front, left, right, wall angle"]
    MPU["MPU6050<br/>heading and angular change"]

    SNAP["Time-stamped sensor snapshots"]
    VALID["Validation and filtering"]
    FSM["40 Hz finite-state controller"]
    ACT["Steering servo and rear motor"]
    TELE["CSV telemetry"]

    CAM --> SNAP
    LIDAR --> SNAP
    MPU --> SNAP
    SNAP --> VALID --> FSM
    FSM --> ACT
    FSM --> TELE


## How each sensor is used

### RPLidar A1

The LiDAR is the main geometric sensor. Its scans are divided into angular sectors to obtain front, left, and right distances. Recent measurements are filtered with medians and checked for minimum point count, age, physical range, and impossible jumps.

It supports:

- Corridor centering before the driving direction is known.
- Outer-wall following during normal sections.
- Selection of the closest reliable wall during pillar avoidance.
- Detection of the front wall before a corner.
- Verification that enough space exists during reverse and corner exit.
- Recovery when a distance becomes critical or the LiDAR stops updating.

### Camera

The IMX219 camera detects red and green traffic pillars. The image pipeline applies a region of interest, color segmentation, blob filtering, and geometric plausibility checks.

For every candidate, the controller considers:

- Color: red or green.
- Horizontal position (x).
- Bottom position in the image.
- Blob area and shape.
- Agreement between two independent distance estimates.

The color determines the legal passing side:

| Pillar | Robot passes on | Pillar remains on |
|---|---|---|
| Red | Right | Left side of the robot |
| Green | Left | Right side of the robot |

A short hold period prevents the maneuver from ending because of one missed camera frame. After the pillar disappears, the controller completes the crossing using LiDAR distance and MPU6050 heading.

### MPU6050

The MPU6050 is calibrated while the robot is completely stationary. The Z-axis gyroscope is filtered and integrated to estimate accumulated heading.

It is used to:

- Maintain a heading reference during cascaded wall control.
- Measure the angular displacement produced while avoiding a pillar.
- Apply counter-steering after the pillar has been passed.
- Confirm each 90-degree corner by heading instead of time alone.
- Compare the expected total heading with the number of completed corners.

Using heading feedback makes the corner maneuver less dependent on motor speed or battery voltage.

## Finite-state machine

mermaid
stateDiagram-v2
    [*] --> START
    START --> DETERMINE_DIRECTION: sensors ready
    DETERMINE_DIRECTION --> REVERSE: first corner detected

    STRAIGHT --> PILLAR: valid pillar detected
    PILLAR --> STRAIGHT: pillar passed
    STRAIGHT --> REVERSE: corner confirmed

    REVERSE --> TURN: reverse clearance reached
    TURN --> EXIT_TURN: 90-degree turn confirmed
    EXIT_TURN --> STRAIGHT: more corners required
    EXIT_TURN --> FINISHED: final corner completed

    DETERMINE_DIRECTION --> RECOVER: recoverable condition
    STRAIGHT --> RECOVER: recoverable condition
    PILLAR --> RECOVER: recoverable condition
    REVERSE --> RECOVER: recoverable condition
    TURN --> RECOVER: recoverable condition
    EXIT_TURN --> RECOVER: recoverable condition
    RECOVER --> DETERMINE_DIRECTION: retry initial approach
    RECOVER --> STRAIGHT: retry section
    RECOVER --> REVERSE: retry corner
    FINISHED --> [*]


| State | Main responsibility | Principal sensors |
|---|---|---|
| START | Initialize actuators, telemetry, and sensor threads | All |
| DETERMINE_DIRECTION | Center the robot and identify the outer wall at the first corner | LiDAR, camera, MPU6050 |
| STRAIGHT | Follow the active reference wall and search for the next event | LiDAR, camera, MPU6050 |
| PILLAR | Cross to the legal side and pass the detected pillar | Camera, LiDAR, MPU6050 |
| REVERSE | Create enough space before turning | LiDAR |
| TURN | Execute and confirm the 90-degree corner | MPU6050, LiDAR fallback |
| EXIT_TURN | Leave the corner and restore normal motion | LiDAR |
| RECOVER | Reverse, reposition, and retry a recoverable state | LiDAR, active-state context |
| FINISHED | Stop motion, close telemetry, and stop threads | All |

## Development process

### 1. Basic propulsion and steering

Development started by calibrating the rear motor, steering center, and asymmetric left/right steering limits. Braking was improved with a short opposite-direction motor pulse to reduce inertia and make stopping distance more repeatable.

### 2. LiDAR wall following

The first autonomous behavior used the RPLidar A1 to follow the outer wall. Raw scans were divided into sectors and filtered. A fixed-rate loop was then introduced so control timing no longer depended on the LiDAR scan rate.

The straight controller evolved into a cascaded structure:

1. Lateral distance error generates a target heading offset.
2. MPU6050 heading error generates the steering command.
3. Steering limits and critical-distance protections constrain the result.

### 3. Camera-based pillar detection

Red and green segmentation was added using the Raspberry Pi camera. Early tests showed that color alone could create false detections, so the detector was expanded with minimum area, aspect ratio, bounding-box fill, region-of-interest, temporal confirmation, and geometric plausibility filters.

Two image measurements estimate pillar distance independently: blob area and vertical bottom position. A candidate is accepted only when both estimates are reasonably consistent.

### 4. Pillar avoidance

The first avoidance controller always referenced the outer wall. That became unreliable when the robot crossed away from it, especially when a black wall produced few LiDAR returns at long range.

The final strategy follows one rule: *during avoidance, use the wall on the side toward which the robot is crossing*. This keeps the reference wall closer and gives the LiDAR a more reliable surface.

The maneuver continues briefly after the camera loses sight of the pillar. LiDAR distance controls the remaining lateral crossing, while the MPU6050 measures the heading change and guides the counter-steering phase.

### 5. MPU6050 heading control

Time-based turns varied with battery voltage and surface conditions. The MPU6050 was therefore added to measure heading directly. The software calibrates gyroscope bias at startup, filters vibration, learns or applies the rotation sign, and confirms the corner when the accumulated change reaches the target angle.

### 6. Recovery and robustness

Instead of aborting after every sensor anomaly, the controller classifies selected faults as recoverable. Recovery brakes the robot, chooses a safe reverse steering direction from the obstacle location, reverses until space is available, centers the steering, and retries the interrupted state.

Additional protections include:

- Sensor age and health checks.
- LiDAR jump rejection held for the complete scan.
- Maximum valid wall distance.
- Critical front and lateral distances.
- Camera outage speed limitation.
- Maximum duration for blind crossing phases.
- Limited recovery and corner retry counts.
- CSV telemetry for post-run analysis.

### 7. Direction detection and complete-lap sequence

At startup, the robot does not yet know which wall is outside. It centers between both walls and approaches the first corner. The side that opens identifies the inner area of the track; the opposite side becomes the outer reference wall. The controller then executes the first corner and repeats straight-section and corner states until the configured number of corners is complete.

## Software requirements

- Raspberry Pi OS with Python 3
- gpiozero
- pigpio daemon and Python interface
- opencv-python / Raspberry Pi OpenCV package
- picamera2
- smbus2
- rplidar (rplidar-roboticia compatible API)


## Running the controller

Start the pigpio service before a hardware run:

bash
sudo systemctl start pigpiod


Run a normal three-lap session:

bash
cd /home/pi/wro2026/wro2026_obstacle
source /home/pi/rplidar_env/bin/activate
python3 wro2026_obstacle_english.py


Run one lap for testing:

bash
python3 wro2026_obstacle_english.py --vueltas 1


Run the logic tests without hardware:

bash
python3 wro2026_obstacle_english.py --autotest


Run the simulator when the companion simulation module is available:

bash
python3 wro2026_obstacle_english.py --sim


Disable telemetry if required:

bash
python3 wro2026_obstacle_english.py --sin-telemetria


> Keep the robot completely still during MPU6050 calibration. For bench testing, raise the driven wheels or disconnect motor power until the steering and sensor checks are complete.

## Telemetry

During a run, the controller can save CSV rows containing:

- Current state and corner number.
- Front, left, and right LiDAR distances.
- Active reference wall and target distance.
- Steering and speed commands.
- Pillar color, horizontal position, bottom position, and area.
- MPU6050 heading and relative heading.
- LiDAR and camera health flags.

These logs were essential for comparing runs, identifying false measurements, adjusting thresholds, and verifying whether a failure came from perception, control, or mechanical behavior.

## Repository structure

text
.
├── README.md
├── wro2026_obstacle_english.py   # Main controller
├── simulacion.py                 # Optional hardware simulator
├── diagrams/
│   ├── flowchart.png
│   └── state_machine.png
└── telemetry/                    # CSV run logs (normally ignored by Git)


The main controller currently imports the optional simulator as simulacion.py. If the file is renamed, update the import in the controller at the same time.

# wro2026_open.py

## Autonomous LiDAR Wall-Following Robot

First version of an autonomous mobile robot controller developed in Python for Raspberry Pi. The robot uses an RPLidar sensor to detect surrounding walls, selects a navigation direction automatically, follows a reference wall with PID steering control, and performs open-loop 90-degree corner maneuvers.

## Project Status

This is validate the main navigation architecture:

- Continuous LiDAR acquisition in a background thread.
- Front, left, and right region-of-interest processing.
- Wall-following control using a discrete PID controller.
- Median-based steering-output filtering.
- Automatic initial direction selection.
- Steering-servo control with mechanical limits.
- Front and rear drivetrain control.
- Automatic corner detection and turning.
- Safe actuator shutdown when the program ends.

## System Overview

The software is divided into four main layers:

1. *Perception:* reads RPLidar scans and separates measurements into front, left, and right zones.
2. *Filtering:* rejects low-quality, out-of-range, and discontinuous measurements.
3. *Control:* calculates a steering correction using a discrete PID controller.
4. *Actuation:* commands the steering servo and the front/rear motors.

## Hardware

- Raspberry Pi 4.
- RPLidar connected through USB.
- Steering servomotor.
- Front DC motor or front axle motor group.
- Rear DC motor or rear axle motor group.
- Two DRV8871 H-bridge motor drivers.
- Independent and properly regulated power supply for the motors and servo.
- Common ground between the Raspberry Pi and motor/servo electronics.

## GPIO Connections

The program uses BCM GPIO numbering.

| Subsystem | Signal | BCM GPIO |
|---|---|---:|
| Front motor driver | IN1 | 17 |
| Front motor driver | IN2 | 18 |
| Rear motor driver | IN1 | 23 |
| Rear motor driver | IN2 | 22 |
| Steering servo | PWM signal | 12 |

The RPLidar serial port is configured as:

text
/dev/ttyUSB0


> Verify the GPIO wiring and voltage levels before powering the robot. Do not power the motors or steering servo directly from the Raspberry Pi GPIO header.

## Software Requirements

- Raspberry Pi OS.
- Python 3.
- A running pigpio daemon.
- RPLidar Python driver.
- GPIO Zero.
- NumPy.
- SciPy.

The project uses the following Python imports:

python
from rplidar import RPLidar
from gpiozero import Motor, AngularServo
import numpy as np
from scipy.ndimage import median_filter


## Python Environment

For the current Raspberry Pi setup, the RPLidar virtual environment is located at:

bash
/home/pi/rplidar_env


Activate it before running the program:

bash
source /home/pi/rplidar_env/bin/activate


Start the pigpio service:

bash
sudo systemctl start pigpiod


You can verify its status with:

bash
sudo systemctl status pigpiod


## Running the Program

1. Connect the RPLidar, steering servo, and motor drivers.
2. Place the robot in a safe test area with the wheels raised for the first test.
3. Start the pigpio daemon.
4. Activate the RPLidar virtual environment.
5. Run the Python file:

bash
cd /path/to/project
source /home/pi/rplidar_env/bin/activate
python3 main.py


Replace main.py with the actual filename if necessary.

Stop the program with Ctrl+C. The shutdown routine stops both motors and returns the steering command to its center position.

## LiDAR Regions of Interest

The point cloud is divided into three angular zones:

| Region | Minimum angle | Maximum angle |
|---|---:|---:|
| Front | -5° | 5° |
| Left | 255° | 285° |
| Right | 75° | 105° |

Only samples with a quality value of at least 5 and a nonzero distance are processed.

## LiDAR Filtering

Each region passes through a filtering pipeline:

1. Sort points by angle.
2. Reject distances outside the configured range.
3. Interpolate invalid samples.
4. Apply a median filter.
5. Reject abrupt distance discontinuities.
6. Select the nearest valid obstacle in each region.

Default filtering parameters:

| Parameter | Value |
|---|---:|
| Minimum distance | 150 mm |
| Maximum distance | 6000 mm |
| Median window | 3 samples |
| Step threshold | 500 mm |

## Steering Configuration

The steering convention is:

- 0°: straight.
- Positive angle: left turn.
- Negative angle: right turn.

Current calibration values:

| Parameter | Value |
|---|---:|
| Mechanical center offset | -20° |
| Maximum steering command | ±40° |
| Servo pulse-width range | 0.5–2.5 ms |

The requested steering command is constrained before the mechanical offset is applied.

## Drivetrain Configuration

The base velocity is:

python
target_velocity = 0.5


Front and rear power can be calibrated independently:

python
FACTOR_FRONT = 1.00
FACTOR_REAR = 1.00


The front axle is currently controlled by:

python
MOTOR_FRONT_FLAG = False


With this value, only the rear motor is actively commanded. Change it to True only after verifying the front motor direction and drivetrain behavior.

## PID Wall Following

The robot follows either the left or right wall at a reference distance of:

text
300 mm


Current PID settings:

| Parameter | Value |
|---|---:|
| Kp | 1.2 |
| Ki | 0.0 |
| Kd | 0.5 |
| Sample period | 0.005 s |
| Steering-output limit | ±40° |

The controller uses the selected wall to determine the error sign:

- IZQUIERDA: follow the left wall.
- DERECHA: follow the right wall.

A five-sample history buffer suppresses sudden steering changes. If a new command differs from the recent median by more than 15°, the median value is used instead.

## Automatic Start Sequence

At startup, the robot moves forward slowly while examining both sides:

- If the left opening is greater than 1000 mm and the front wall is at or below 750 mm, the robot turns left and begins following the right wall.
- If the right opening is greater than 1000 mm and the front wall is at or below 750 mm, the robot turns right and begins following the left wall.

## Corner Detection and Turning

During the main wall-following segment, a corner condition is detected when:

- The front distance is at or below 725 mm.
- More than 250 control cycles have elapsed.
- The active segment was configured for 10000 cycles.

The corner label CUELLO is added to the telemetry list, and the controller exits the current straight segment.

The robot then executes a 90-degree turn using:

- Maximum steering angle: 40°.
- Open-loop turn duration: 0.95 s.

These values must be calibrated on the actual surface, battery level, speed, and steering geometry.

## Main Navigation Sequence

The current main routine:

1. Initializes the actuators.
2. Starts the asynchronous LiDAR worker.
3. Waits six seconds for scan stabilization.
4. Determines the initial direction.
5. Repeats wall following and corner turning for 11 segments.
6. Executes a final 300-cycle wall-following segment.
7. Stops the drivetrain and centers the steering during shutdown.

## Main Functions

| Function | Purpose |
|---|---|
| set_steering_angle() | Constrains and applies the steering command |
| drive_forward_awd() | Drives the enabled axles forward |
| drive_backward_awd() | Drives the enabled axles backward |
| stop_awd() | Stops both motors |
| filter_lidar_zone() | Filters one LiDAR region |
| lidar_worker_thread() | Reads and processes LiDAR scans asynchronously |
| sanitize_control_signal() | Rejects sudden steering-output changes |
| pid_wall_follower() | Performs closed-loop wall following |
| execute_start_sequence() | Selects the initial navigation direction |
| execute_90deg_left_turn() | Performs an open-loop left turn |
| execute_90deg_right_turn() | Performs an open-loop right turn |
| initialize_system() | Sets safe initial actuator states |
| terminate_system() | Stops and centers all actuators |

## Known Limitations

- The program assumes that valid LiDAR values are available before every comparison.
- Missing side or front readings may produce runtime errors.
- The LiDAR recovery loop does not currently recreate the sensor object after every failure.
- Turns are time-based rather than closed-loop.
- The telemetry list is stored in memory but is not exported.
- The declared sample period may differ from the real execution period.
- The program does not yet include a physical start button.
- The front axle is disabled by default.
- Competition-specific start, lap-counting, parking, and fail-safe requirements still need validation.

## Planned Improvements

- Add explicit handling for missing or stale LiDAR measurements.
- Reconnect the RPLidar automatically after communication failures.
- Use monotonic timing for the PID loop.
- Implement anti-windup and measured-time PID calculations.
- Replace time-based turns with sensor- or IMU-assisted control.
- Save telemetry to a CSV file.
- Add a physical start button and competition start delay.
- Add watchdog and emergency-stop behavior.
- Divide perception, control, hardware, and main logic into separate modules.
- Add repeatable calibration and validation procedures.
