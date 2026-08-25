import threading
from rplidar import RPLidar
import os
# Force pigpio factory for hardware-timed, jitter-free PWM signals
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'
from gpiozero import Motor, AngularServo
import time
import numpy as np
from scipy.ndimage import median_filter
import logging
from collections import deque

# ==============================================================================
#    GPIO PIN CONFIGURATION
# ==============================================================================
# AWD Drivetrain (DRV8871 H-Bridge Drivers)
PIN_FRONT_IN1 = 17
PIN_FRONT_IN2 = 18
PIN_REAR_IN1 = 23
PIN_REAR_IN2 = 22
MOTOR_FRONT_FLAG = False  # Toggle front-axle engagement for efficiency/testing

# Steering Actuator
PIN_SERVO_SIGNAL = 12

# ==============================================================================
#    HARDWARE CALIBRATION & PARAMETERS
# ==============================================================================
# 1. AWD Drivetrain Power Balancing:
FACTOR_FRONT = 1.00  # Front axle power scaling coefficient (0.0 to 1.0)
FACTOR_REAR  = 1.00  # Rear axle power scaling coefficient (0.0 to 1.0)

# 2. Steering Kinematics Calibration:
CENTER_OFFSET = -20   # Mechanical alignment offset (negative value trims right)
STEERING_LIMIT = 40  # Maximum permissible steering angle to prevent binding

# ==============================================================================
#    HARDWARE INITIALIZATION
# ==============================================================================
# H-Bridge DC Motor Actuators
motor_front = Motor(forward=PIN_FRONT_IN1, backward=PIN_FRONT_IN2)
motor_rear  = Motor(forward=PIN_REAR_IN1,  backward=PIN_REAR_IN2)

# High-Resolution PWM Steering Servo
steering = AngularServo(
    12, 
    min_angle=-90, 
    max_angle=90, 
    min_pulse_width=0.0005, 
    max_pulse_width=0.0025, 
    initial_angle=0
)

# ==============================================================================
#    LIDAR SENSOR CONFIGURATION
# ==============================================================================
PORT_NAME = "/dev/ttyUSB0"

# Spatial partitioning thresholds (in degrees) to segment point clouds
ANGULO_FRONT_MIN = -5
ANGULO_FRONT_MAX = 5

ANGULO_LEFT_MIN = 255
ANGULO_LEFT_MAX = 285

ANGULO_RIGHT_MIN = 75
ANGULO_RIGHT_MAX = 105

# --- Thread-Safe Global State Variables per Region of Interest (ROI) ---
lidar_distance_front = None
lidar_distance_left = None
lidar_distance_right = None

lidar_running = True
lidar_lock = threading.Lock()  # Mutex lock to guarantee thread safety and prevent race conditions

# ==============================================================================
#    GLOBAL NAVIGATION STATE
# ==============================================================================
control_direction = ""  # Defines active reference wall ("LEFT" or "RIGHT")
target_velocity = 0.5   # Base duty cycle for drivetrain speed control
telemetry_data = []     # Historical log array for post-run analysis
control_history = deque(maxlen=5)  # Circular buffer for discrete signal filtering

# ==============================================================================
#    STEERING CONTROL SUBSYSTEM
# ==============================================================================
def set_steering_angle(desired_angle):
    """
    Executes constrained geometric steering control.
    Convention: 0° = Rectilinear, Positive = Left Turn, Negative = Right Turn.
    """
    # 1. Structural bounding box to protect mechanical linkages
    safe_angle = min(max(desired_angle, -STEERING_LIMIT), STEERING_LIMIT)
    
    # 2. Apply linear mapping offset for zero-point chassis calibration
    final_angle = safe_angle + CENTER_OFFSET
    
    # 3. Dispatched hardware command via PWM duty cycle
    steering.angle = final_angle

# ==============================================================================
#    DRIVETRAIN ACTUATION SUBSYSTEM
# ==============================================================================
def drive_forward_awd(base_velocity):
    """Propels the vehicle forward scaling current duty cycles by axle-specific constants."""
    vel_front = min(max(base_velocity * FACTOR_FRONT, 0.0), 1.0)
    vel_rear  = min(max(base_velocity * FACTOR_REAR, 0.0), 1.0)
    
    if MOTOR_FRONT_FLAG:
        motor_front.forward(speed=vel_front)
    motor_rear.forward(speed=vel_rear)

def drive_backward_awd(base_velocity):
    """Reverses the vehicle using the calibrated traction balancing model."""
    vel_front = min(max(base_velocity * FACTOR_FRONT, 0.0), 1.0)
    vel_rear  = min(max(base_velocity * FACTOR_REAR, 0.0), 1.0)
    
    if MOTOR_FRONT_FLAG:
        motor_front.backward(speed=vel_front)
    motor_rear.backward(speed=vel_rear)

def stop_awd():
    """Forces an immediate zero-power electronic stop across the AWD system."""
    motor_front.stop()
    motor_rear.stop()
    
# ==============================================================================
#    DIGITAL SIGNAL PROCESSING & PERCEPTION (LIDAR)
# ==============================================================================
def filter_lidar_zone(point_cloud_roi, dist_min=150.0, dist_max=6000.0, window_size=3, step_threshold=500.0):
    """
    Vectorized Digital Signal Processing (DSP) pipeline optimized using NumPy.
    Smooths data, handles outlier reflection rejections, and avoids CPU blocking.
    """
    n_points = len(point_cloud_roi)
    if n_points < window_size:
        return point_cloud_roi

    # 1. High-speed array conversion and quick-sorting by heading
    raw_data = np.array(point_cloud_roi)  # Structure: [[distance, angle], ...]
    raw_data = raw_data[raw_data[:, 1].argsort()]

    distances = raw_data[:, 0]
    angles = raw_data[:, 1]

    # 2. Hard Threshold Bandpass Range Filter
    valid_range_mask = (distances >= dist_min) & (distances <= dist_max)
    if not np.any(valid_range_mask):
        return []

    # Linear interpolation to substitute out-of-range sensor dropouts safely
    filtered_distances = np.interp(
        np.arange(n_puntos:=n_points),
        np.where(valid_range_mask)[0],
        distances[valid_range_mask],
    )

    # 3. Non-linear Median Filtering to eliminate specular multi-path reflections
    filtered_distances = median_filter(filtered_distances, size=window_size, mode="wrap")

    # 4. Step Discontinuity Mitigation (First-order derivative filtering)
    gradients = np.abs(np.diff(filtered_distances, prepend=filtered_distances[-1]))
    stable_gradient_mask = gradients <= step_threshold

    # Compound boolean conditional index mask
    optimized_mask = valid_range_mask & stable_gradient_mask

    # 5. Native reconstruction recovery for consumer loops
    return list(zip(filtered_distances[optimized_mask], angles[optimized_mask]))


def lidar_worker_thread():
    """
    Asynchronous sensor worker thread. Continuously processes telemetry streams,
    parses geographic quadrants, and populates global state arrays via mutex locks.
    """
    global lidar_running
    global lidar_distance_front, lidar_distance_left, lidar_distance_right

    lidar = None
    lidar = RPLidar(PORT_NAME, timeout=3)
    lidar._serial.reset_input_buffer()
    
    # Suppress low-level rplidar driver logs to avoid console pollution
    logging.getLogger("rplidar").setLevel(logging.CRITICAL)
    lidar.start_motor()
    time.sleep(1)  # Allow hardware rotational stabilization latency

    while lidar_running:
        try:
            for scan in lidar.iter_scans(max_buf_meas=250, min_len=5):
                if not lidar_running:
                    break

                puntos_front = []
                puntos_left = []
                puntos_right = []

                # Low-level point classification & hardware validation filter
                for quality, angle, distance in scan:
                    if quality < 5:    # Discard low-intensity optical return pulses
                        continue
                    if distance == 0:  # Exclude hardware null-returns
                        continue

                    if ANGULO_FRONT_MIN <= angle <= ANGULO_FRONT_MAX:
                        puntos_front.append((distance, angle))
                    elif ANGULO_LEFT_MIN <= angle <= ANGULO_LEFT_MAX:
                        puntos_left.append((distance, angle))
                    elif ANGULO_RIGHT_MIN <= angle <= ANGULO_RIGHT_MAX:
                        puntos_right.append((distance, angle))

                # --- ADVANCED DSP PIPELINE APPLICATION ---
                # dist_min is matched to the outer chassis geometry to prevent self-mapping
                puntos_front = filter_lidar_zone(puntos_front, dist_min=150.0)
                puntos_left = filter_lidar_zone(puntos_left, dist_min=150.0)
                puntos_right = filter_lidar_zone(puntos_right, dist_min=150.0)

                # Mutex transaction lock block to prevent memory segmentation faults or data races
                with lidar_lock:
                    if puntos_front:
                        d, a = min(puntos_front, key=lambda x: x[0])
                        lidar_distance_front = d
                    if puntos_left:
                        d, a = min(puntos_left, key=lambda x: x[0])
                        lidar_distance_left = d
                    if puntos_right:
                        d, a = min(puntos_right, key=lambda x: x[0])
                        lidar_distance_right = d
                
        except Exception as e:
            print("LIDAR RUNTIME EXCEPTION:", e)
        finally:
            try:
                if lidar:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
            except:
                pass
        if lidar_running:
            time.sleep(1)  # Adaptive back-off before attempting driver re-instantiation

# ==============================================================================
# ==============================================================================
# OPEN CHALLENGE MANEUVER ROUTINES
# ==============================================================================
def execute_90deg_left_turn():
    """Executes a blind open-loop Left turn maneuver based on calculated kinematic runtime."""
    set_steering_angle(40)
    time.sleep(0.95)
    set_steering_angle(0)

def execute_90deg_right_turn():
    """Executes a blind open-loop Right turn maneuver based on calculated kinematic runtime."""
    set_steering_angle(-40)
    time.sleep(0.95)
    set_steering_angle(0)

# ==============================================================================
# CLOSED-LOOP CONTROL THEORY (PID)
# ==============================================================================
def sanitize_control_signal(current_value, jitter_threshold=15.0, reset_buffer=False):
    """
    Implements a statistical outlier filter on the control output array.
    Prevents step-jumps and protects physical actuators from instantaneous reversals.
    """
    global control_history
    
    if reset_buffer:
        control_history.clear()
        return current_value
        
    if len(control_history) == 0:
        for _ in range(control_history.maxlen):
            control_history.append(current_value)
        return current_value

    # Evaluate running central tendency trend
    historical_median = np.median(control_history)

    # If the candidate output introduces high-frequency noise, damp it out using the rolling median
    if abs(current_value - historical_median) > jitter_threshold:
        sanitized_value = historical_median
    else:
        sanitized_value = current_value

    control_history.append(sanitized_value)
    return sanitized_value

def pid_wall_follower(reference_distance, execution_cycles, velocity, selected_sensor):
    """
    Velocity-form Closed-Loop Parallel Proportional-Integral-Derivative (PID) Controller.
    Dynamically aligns the vehicle relative to structural boundaries.
    """
    global lidar_distance_front, lidar_distance_left, lidar_distance_right
    global telemetry_data
    
    output = error = p_error = p_error_2 = 0.0
    kp, ki, kd = 1.2, 0.0, 0.5  # Tuned heuristics for optimized convergence rate
    T = 0.005  # Discrete sample time period (200Hz Control Loop Execution)
    p_output = 0.0
    cycle_counter = 0
    
    sanitize_control_signal(0, reset_buffer=True)
    drive_forward_awd(velocity)
    
    while cycle_counter <= execution_cycles:
        p_error_2 = p_error
        p_error = error
        
        # State machine sensor selection for feedback path loop closing
        if selected_sensor == "IZQUIERDA":  # Tracking Left Wall
            sensor_input = lidar_distance_left
            error = reference_distance - sensor_input
        elif selected_sensor == "DERECHA":  # Tracking Right Wall
            sensor_input = lidar_distance_right
            error = sensor_input - reference_distance
            
        # Standard Discrete-Time PID Positional Formulation
        output = int(kp * (error - p_error) + (ki * error) + (kd * (error - (2 * p_error) + p_error_2)) + p_output)
        output = min(max(output, -40), 40)  # Constrain output to safe steering boundaries
        
        sanitized_output = sanitize_control_signal(output)
        set_steering_angle(0 - sanitized_output)
        
        p_output = sanitized_output
        cycle_counter += 1
        
        # Strategic early exit check: Critical path bottleneck ("CUELLO") discovery condition
        if lidar_distance_front <= 725 and cycle_counter > 250 and execution_cycles == 10000:
            telemetry_data.append("CUELLO")
            break
        time.sleep(T)

# ==============================================================================
# INITIALIZATION STATE MACHINE
# ==============================================================================
def execute_start_sequence():
    """
    Autonomous boot routine. Analyzes initial boundary spacing to programmatically
    determine tracking priorities and orientation choices for the mapping track.
    """
    global lidar_distance_front, lidar_distance_left, lidar_distance_right
    global control_direction, target_velocity
    direction_resolved = False
    
    drive_forward_awd(target_velocity - 0.3)  # Creep forward safely while analyzing environment
    
    while not direction_resolved:
        if lidar_distance_left is not None:
            # Blind corner or left entry gap mapping detected
            if lidar_distance_left > 1000 and lidar_distance_front <= 750:
                drive_forward_awd(target_velocity)
                execute_90deg_left_turn()
                direction_resolved = True
                control_direction = "DERECHA"  # Transition to close-range Right Wall reference lock
                
        if lidar_distance_right is not None:
            # Blind corner or right entry gap mapping detected
            if lidar_distance_right > 1000 and lidar_distance_front <= 750:
                drive_forward_awd(target_velocity)
                execute_90deg_right_turn()
                direction_resolved = True
                control_direction = "IZQUIERDA"  # Transition to close-range Left Wall reference lock
        time.sleep(0.1)

# ==============================================================================
# SYSTEM SETUP & TEARDOWN FUNCTIONS
# ==============================================================================
def initialize_system():
    """Pre-flight setup: Ensures actuator safety states are zeroed before processing."""
    stop_awd()
    set_steering_angle(0)
    print("[INFO] Embedded Subsystems Initialized Successfully...")

def terminate_system():
    """Failsafe teardown sequence: Safely handles emergency deceleration protocols."""
    stop_awd()
    set_steering_angle(0)
    print("[INFO] Actuators Safely Isolated. System Standby.")
    time.sleep(1)

# ==============================================================================
# MAIN EXECUTIVE ROUTINE LOOP
# ==============================================================================
if __name__ == "__main__":
    try:
        initialize_system()
        print("[THREADING] Dispatching Asynchronous Perception Worker...")
        threading.Thread(target=lidar_worker_thread, daemon=True).start()
        
        time.sleep(6)  # Spin-up delay to allow structural point-clouds to stabilize
        execute_start_sequence()
        
        lap_counter = 1
        while lap_counter <= 11:
            # High-priority continuous PID lane centered navigation run
            pid_wall_follower(300, 10000, target_velocity, control_direction)
            
            # Corner Negotiation Phase transitions
            if control_direction == "DERECHA":
                execute_90deg_left_turn()
            else:
                execute_90deg_right_turn()
            lap_counter += 1
            
        # Final stretch braking approach execution run
        pid_wall_follower(300, 300, target_velocity, control_direction)
        
    except KeyboardInterrupt:
        print("\n[USER INTERRUPT] Aborting run...")
        lidar_running = False
        terminate_system()
    finally:
        lidar_running = False
        terminate_system()


