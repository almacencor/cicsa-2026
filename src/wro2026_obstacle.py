#!/usr/bin/env python3
"""WRO 2026 Future Engineers obstacle challenge control program.

Hernández González Sergio Amid
Pereida Arochi Diego
Garcia Audelo Gildardo

The robot uses concurrent LiDAR, camera, and MPU6050 acquisition threads and
a fixed-rate finite-state controller for wall following, pillar avoidance,
cornering, recovery, and telemetry.
"""
import argparse
import csv
import inspect
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from statistics import median
from typing import Deque, List, Optional, Tuple

SIM = "--sim" in sys.argv
AUTOTEST = "--autotest" in sys.argv

if not SIM and not AUTOTEST:
    os.environ["GPIOZERO_PIN_FACTORY"] = "pigpio"
                                                                  
# General configuration and hardware pins
VERSION = "V6.3-CENTRADO-ARRANQUE"
DEBUG = True
PORT_NAME = "/dev/ttyUSB0"

                                                                         
LAPS = 3
CORNERS_POR_LAP = 4
SECTION_FINAL_S = 0.0                                                         

PIN_MOTOR_IN1 = 23
PIN_MOTOR_IN2 = 22
PIN_SERVO = 12
                                                              
OFFSET_CENTER = -20.0
LIMIT_STEERING_LEFT_DEG = 37.0
LIMIT_STEERING_RIGHT_DEG = 33.0
SERVO_BAND_DEADBAND_DEG = 0.5                                            
                                                                  
BRAKE_PULSE_S = 0.07
BRAKE_SPEED = 0.35
MOTOR_PWM_HZ = None                                                                       
                                                                          
CONTROL_PERIOD_S = 0.025                                           
DISTANCE_LATERAL_TARGET_MM = 430.0
KP_LATERAL = 0.025                                                  
KA_YAW = 0.0                                                                
                                                                                                                                            
KP_DIST_A_HEADING = 0.090                                                 
HEADING_OFFSET_MAX_DEG = 15.0
                                                                          
KI_DIST_A_HEADING = 0.0                                                         
HEADING_INTEGRAL_MAX_DEG = 12.0
KA_HEADING = 1.10                                                               
KD_LATERAL = 0.004                                                             
DERIVADA_MAX_DT_S = 0.35
STEERING_CONTROL_MAX_DEG = 12.0
STEERING_ESCAPE_DEG = 24.0
DISTANCE_ESCAPE_MM = 300.0
DISTANCE_CRITICA_MM = 155.0
DISTANCE_CRITICA_CONFIRMATIONS = 3
                                                                                                            
WALL_MAX_VALIDA_MM = 950.0
WALL_LOST_TIMEOUT_S = 1.5
                                                                                                                             
CARRIL_BLIND_TIMEOUT_S = 2.0
                              
CENTRADO_KP_DEG_POR_MM = 0.020
CENTRADO_MAX_DEG = 10.0

SPEED_STRAIGHT = 0.30
SPEED_APPROACH = 0.20
SPEED_ESCAPE = 0.20
SPEED_EXIT = 0.20

DISTANCE_FRENAR_MM = 950.0
DISTANCE_INICIAR_STEERING_MM = 620.0
FRONT_CRITICAL_MM = 190.0
CONFIRMATIONS_CORNER = 2
TIME_MIN_SECTION_S = 1.0
DISTANCE_DESPEJADO_MM = 800.0                                                                            
                                                              
REVERSE_ANGLE_DEG = 26.0
REVERSE_SPEED = 0.22
REVERSE_FRONT_TARGET_MM = 620.0
REVERSE_TIME_MIN_S = 0.60
REVERSE_TIME_MAX_S = 1.40
STOP_ANTES_DIRECTION_S = 0.35
WAIT_SERVO_REVERSE_S = 0.18
STOP_CAMBIO_GEAR_S = 0.20

                                                                    
STEERING_TARGET_DEG = 85.0
STEERING_SUAVIZAR_DESDE_DEG = 65.0
STEERING_ANGLE_DEG = 32.0
STEERING_ANGLE_FINAL_DEG = 20.0
STEERING_SPEED = 0.23
STEERING_TIME_MIN_S = 0.80
STEERING_TIME_MAX_S = 3.20
STEERING_FRONT_LIBRE_MM = 850.0
STEERING_FRONT_STOP_MM = 220.0
EXIT_STRAIGHT_S = 0.35
MAX_ATTEMPTS_CORNER = 3

                                                               
WALL_NEW_MIN_MM = 200.0
WALL_NEW_MAX_MM = 780.0
WALL_NEW_STABLE_DELTA_MM = 55.0
WALL_NEW_CONFIRMATIONS = 3
WALL_NEW_JUMP_MM = 150.0
WALL_LOST_MIN_S = 0.45
EXIT_CERCA_WALL_MM = 320.0
EXIT_ESCAPE_DEG = 12.0

                                                                             
MAX_RECUPERACIONES = 5
RECOVER_REVERSE_SPEED = 0.20
RECOVER_REVERSE_MAX_S = 1.30
RECOVER_FRONT_LIBRE_MM = 850.0
RECOVER_PAUSA_S = 0.25
                                                                                                                                                  
RECOVER_ANGLE_DEG = 26.0
                                                                                                                          
RECOVER_EXTRA_POR_INTENTO_S = 0.35
                                                              
JUMP_MAX_MM_POR_S = 900.0
                                                                                                                                                   
JUMP_CICLOS_MAX = 6
                                                                             
LIDAR_TIMEOUT_S = 0.85
LIDAR_RETRY_S = 0.8
QUALITY_MIN = 5
DIST_MIN_MM = 100.0
DIST_MAX_MM = 6000.0
LIDAR_OFFSET_DEG = -3.0                                                   
SECTOR_MIN_PUNTOS = 1

FRONT_MIN_DEG = -10.0
FRONT_MAX_DEG = 10.0
REAR_BLIND_MIN_DEG = 130.0
REAR_BLIND_MAX_DEG = 230.0
                                                                        
WALL_THETA_DEG = 25.0
DER_PERP_MIN, DER_PERP_MAX = 85.0, 95.0
DER_FWD_MIN, DER_FWD_MAX = 60.0, 70.0
IZQ_PERP_MIN, IZQ_PERP_MAX = 265.0, 275.0
IZQ_FWD_MIN, IZQ_FWD_MAX = 290.0, 300.0
                                                                         
DER_ANCHO_MIN, DER_ANCHO_MAX = 70.0, 110.0
IZQ_ANCHO_MIN, IZQ_ANCHO_MAX = 250.0, 290.0
YAW_MAX_VALIDO_DEG = 45.0

HISTORY = 3
SECTOR_MAX_AGE_S = 0.35
                                                                            
CAM_W, CAM_H, CAM_FPS = 320, 240, 30
CAMARA_FOCAL_PX = 390.0                                                 
CAM_MAX_AGE_S = 0.35
CAM_AVISO_PERIOD_S = 1.0
VISION_MIN_AREA_PX = 120.0
VISION_CONFIRM_FRAMES = 2
VISION_ALTO_MIN_PX = 6
VISION_ASPECTO_MIN = 0.85                                                                                                         
                                                                           
VISION_FILL_MIN = 0.55
ROI_PILARES = (0.00, 1.00)                                      

HSV_RED1_LO, HSV_RED1_HI = (0, 120, 100), (8, 255, 255)
HSV_RED2_LO, HSV_RED2_HI = (170, 120, 100), (179, 255, 255)
HSV_GREEN_LO, HSV_GREEN_HI = (60, 90, 65), (95, 255, 255)

PILAR_DIST_INICIO_MM = 900.0
PILAR_DIST_REFERENCE_MM = 350.0
PILAR_DIST_STOP_MM = 150.0
PILAR_BOTTOM_INICIO = 0.20
PILAR_BOTTOM_MAX = 0.55
PILAR_STEERING_MIN_DEG = 14.0
PILAR_STEERING_MAX_DEG = 24.0
PILAR_SPEED = 0.20
PILAR_HOLD_RED_S = 0.40
PILAR_HOLD_GREEN_S = 0.35
PILAR_STEERING_EXTRA_GREEN_DEG = 3.0
PILAR_ACTIVATION_BOTTOM = 0.15
PILAR_ACTIVATION_AREA_PX = 250.0
                                                      
PILAR_K_AREA_MM = 24100.0
PILAR_K_BOTTOM_MM = 222.0
PILAR_B_HORIZONTE = 0.165                                                                   
                                                                 
PILAR_B_MINIMUM = 0.20
PILAR_RAZON_MIN = 0.65
PILAR_RAZON_MAX = 1.45
                                                       
PILAR_RAZON_BOTTOM_MAX = 0.75
                                            
PILAR_FRONT_X_MAX = 0.25                                                    
PILAR_FRONT_BOTTOM_MIN = 0.45                                                   
                                                                     
PILAR_TARGET_MM = 260.0                                                                           
                                                                 
PILAR_SIGNAL_MAX_MM = 450.0
                                             
PILAR_HEADING_OFFSET_MAX_DEG = 26.0                                                                         
                   
LIDAR_ANGLE_BLIND_DEG = 20.0                                                                           
                                                                
LIDAR_ANGLE_MAX_USABLE_DEG = 29.0                                                                        
                                        
PILAR_KP_DIST_A_HEADING = 0.20
PILAR_STEERING_CASCADA_MAX_DEG = 28.0                                                                          
                                                           
PILAR_KA_HEADING = 2.5
PILAR_STEERING_LIMITADO_DEG = 8.0                                                                           
                                                                       
PILAR_X_TARGET_RED = 0.25
PILAR_X_TARGET_GREEN = 0.75
PILAR_KP_X = 26.0
PILAR_ADJUSTMENT_MAX_DEG = 10.0
PILAR_X_PASSED_RED = 0.12
PILAR_X_PASSED_GREEN = 0.88                                                                           
                                                
PILAR_REFRACTORY_S = 1.60                                                                            
                                                                             
PILAR_EXIT_SPEED = 0.24                                                               
                                      
PILAR_GAP_MINIMUM_MM = 140.0
PILAR_DISTANCE_CRITICA_MM = 105.0
                                                                
COUNTERSTEER_DEG = 16.0
RECOVERY_SPEED = 0.14
                                                                            
MPU_BUS_I2C = 1
MPU_DIRECCIONES = (0x68, 0x69)
MPU_SAMPLES_CALIBRATION = 1000
MPU_INTERVAL_CALIBRATION_S = 0.002
MPU_FACTOR_GYRO = 131.0
MPU_ZONE_DEADBAND_DPS = 0.35
MPU_FILTER_ALFA = 0.70
MPU_PERIOD_S = 0.005
MPU_DELTA_MIN_EVASION_DEG = 3.0
MPU_TOLERANCE_HEADING_DEG = 1.5
MPU_HEADING_CONFIRMATIONS = 1
MPU_RECOVERY_TIMEOUT_S = 5.00
MPU_COUNTERSTEER_MAX_DEG = 24.0
MPU_COUNTERSTEER_LENTO_DEG = 12.0
MPU_COUNTERSTEER_LENTO_THRESHOLD_DEG = 8.0
                                                                                                                                                     
MPU_SIGN_FIXED: Optional[float] = None
MPU_SIGN_THRESHOLD_DEG = 8.0
VERIFICACION_HEADING_TOLERANCE_DEG = 25.0
                                                                            
TELEMETRIA_ENABLED = True
TELEMETRIA_DIR = "telemetria"
TELEMETRIA_FLUSH_ROWS = 40
                                                                             
if AUTOTEST:
    cv2 = None
    AngularServo = Motor = Picamera2 = RPLidar = SMBus = None
elif SIM:
    from simulacion import (                              
        AngularServo,
        Motor,
        Picamera2,
        RPLidar,
        SMBus,
        mundo,
    )
    cv2 = None
    MPU_SAMPLES_CALIBRATION = 60
else:
    import cv2
    from gpiozero import AngularServo, Motor
    from picamera2 import Picamera2
    from rplidar import RPLidar

    try:
        from smbus2 import SMBus
    except ImportError:
        SMBus = None


NUM_CORNERS = LAPS * CORNERS_POR_LAP

LEFT = "IZQUIERDA"
RIGHT = "DERECHA"


# Finite-state machine states recorded in telemetry
class State(Enum):
    """Code-specific behavior for Estado."""

    START = auto()
    DETERMINE_DIRECTION = auto()
    STRAIGHT = auto()
    PILLAR = auto()
    REVERSE = auto()
    TURN = auto()
    EXIT_TURN = auto()
    RECOVER = auto()
    FINISHED = auto()


class RecoverableError(Exception):
    """Code-specific behavior for Recuperable."""

    def __init__(self, message: str, wall: Optional[str] = None) -> None:
        super().__init__(message)
        self.wall = wall


class Fatal(Exception):
    """Code-specific behavior for Fatal."""


@dataclass(frozen=True)
class Scan:
    front: Optional[float]
    left: Optional[float]
    right: Optional[float]
    yaw_left: Optional[float]
    yaw_right: Optional[float]
    healthy: bool
    seq: int
    age: float


@dataclass(frozen=True)
class Vision:
    color: Optional[str]
    x: Optional[float]
    bottom: float
    area: float
    healthy: bool
    age: float
    seq: int


                                                                             
# Select the mandatory passing side from the pillar color
def evasion_side(color: str) -> str:
    """Code-specific behavior for lado evasion."""
    return LEFT if color == "GREEN" else RIGHT


def reference_wall(color: Optional[str], outer_wall: str) -> str:
    """Code-specific behavior for pared de referencia."""
    return outer_wall if color is None else evasion_side(color)


def is_outer_wall(color: Optional[str], outer_wall: str) -> bool:
    """Code-specific behavior for es pared outer."""
    return reference_wall(color, outer_wall) == outer_wall


def rules_table() -> List[Tuple[str, str, str, str]]:
    """Code-specific behavior for tabla reglamento."""
    filas = []
    for direction, outer_wall in (("clockwise", LEFT),
                                    ("anticlockwise", RIGHT)):
        for color in ("GREEN", "RED"):
            ref = reference_wall(color, outer_wall)
            etiqueta = "outer" if ref == outer_wall else "inner"
            filas.append((direction, color, ref, etiqueta))
    return filas


                                                                             
# Keep the control loop independent of sensor update rates
class FixedRateLoop:
    """Code-specific behavior for Ritmo."""

    def __init__(self, period: float) -> None:
        self.period = period
        self._siguiente = time.monotonic()
        self.overruns = 0

    def reset(self) -> None:
        self._siguiente = time.monotonic()

    def wait(self) -> float:
        self._siguiente += self.period
        now = time.monotonic()
        remaining = self._siguiente - now
        if remaining > 0:
            time.sleep(remaining)
            return self._siguiente
        if remaining < -self.period:
            self.overruns += 1
            self._siguiente = now
        return now


                                                                             
# Store time-stamped control and sensor data in CSV format
class Telemetry:
    """Code-specific behavior for Telemetria."""

    CAMPOS = [
        "t", "estado", "esquina", "d", "ref", "obj", "d_izq", "d_der",
        "front", "yaw", "cmd", "vel", "pilar", "pilar_x", "pilar_b",
        "rumbo", "rumbo_rel", "lidar_ok", "vision_ok",
    ]

    def __init__(self) -> None:
        self._archivo = None
        self._writer = None
        self._filas = 0
        self._t0 = time.monotonic()
        self.ruta: Optional[str] = None

    def open(self) -> None:
        if not TELEMETRIA_ENABLED:
            return
        try:
            os.makedirs(TELEMETRIA_DIR, exist_ok=True)
            marca = time.strftime("%Y%m%d_%H%M%S")
            self.ruta = os.path.join(TELEMETRIA_DIR, f"run_{marca}.csv")
            self._archivo = open(self.ruta, "w", newline="")
            self._writer = csv.writer(self._archivo)
            self._writer.writerow(self.CAMPOS)
            print(f"TELEMETRIA: {self.ruta}")
        except Exception as exc:
            print("TELEMETRY UNAVAILABLE:", exc)
            self._archivo = None
            self._writer = None

    def write_row(self, **kwargs) -> None:
        if self._writer is None:
            return
        kwargs.setdefault("t", round(time.monotonic() - self._t0, 3))
        try:
            self._writer.writerow(
                [kwargs.get(campo, "") for campo in self.CAMPOS]
            )
            self._filas += 1
            if self._filas % TELEMETRIA_FLUSH_ROWS == 0:
                self._archivo.flush()
        except Exception:
            pass

    def close(self) -> None:
        if self._archivo is not None:
            try:
                self._archivo.flush()
                self._archivo.close()
                print(f"TELEMETRIA GUARDADA: {self.ruta} ({self._filas} filas)")
            except Exception:
                pass
            self._archivo = None
            self._writer = None


tele = Telemetry()


                                                                             
# Integrate gyroscope Z-axis rate to estimate robot heading
class MPU6050:
    """Code-specific behavior for MPU6050."""

    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_GYRO_XOUT_H = 0x43
    REG_PWR_MGMT_1 = 0x6B
    REG_WHO_AM_I = 0x75

    def __init__(self) -> None:
        self._bus = None
        self._address: Optional[int] = None
        self._offset_z = 0.0
        self._filtered_z = 0.0
        self._heading = 0.0
        self._last_time = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ready = False
        self._error: Optional[str] = None
        self.signo: Optional[float] = MPU_SIGN_FIXED

    @staticmethod
    def _int16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        return value - 65536 if value & 0x8000 else value

    def _read_z(self) -> float:
        if self._bus is None or self._address is None:
            raise RuntimeError("MPU6050 no inicializado")
        data = self._bus.read_i2c_block_data(
            self._address, self.REG_GYRO_XOUT_H, 6
        )
        return self._int16(data[4], data[5]) / MPU_FACTOR_GYRO

    def start(self) -> bool:
        if SMBus is None:
            self._error = "falta python3-smbus2"
            print("MPU6050 UNAVAILABLE:", self._error)
            return False
        try:
            self._bus = SMBus(MPU_BUS_I2C)
            for address in MPU_DIRECCIONES:
                try:
                    ident = self._bus.read_byte_data(address, self.REG_WHO_AM_I)
                    if ident in MPU_DIRECCIONES:
                        self._address = address
                        break
                except OSError:
                    continue
            if self._address is None:
                raise RuntimeError("no responde en 0x68 ni 0x69")

            self._bus.write_byte_data(self._address, self.REG_PWR_MGMT_1, 0x80)
            time.sleep(0.10)
            self._bus.write_byte_data(self._address, self.REG_PWR_MGMT_1, 0x01)
            self._bus.write_byte_data(self._address, self.REG_SMPLRT_DIV, 0x09)
            self._bus.write_byte_data(self._address, self.REG_CONFIG, 0x03)
            self._bus.write_byte_data(self._address, self.REG_GYRO_CONFIG, 0x00)
            time.sleep(0.20)

            print(f"MPU6050 0x{self._address:02X}: calibrando; robot inmovil")
            total = 0.0
            for _ in range(MPU_SAMPLES_CALIBRATION):
                total += self._read_z()
                time.sleep(MPU_INTERVAL_CALIBRATION_S)
            self._offset_z = total / MPU_SAMPLES_CALIBRATION

            with self._lock:
                self._filtered_z = 0.0
                self._heading = 0.0
                self._last_time = time.monotonic()
                self._ready = True
                self._error = None
            self._stop.clear()
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()
            print(f"MPU6050 LISTO | offset Z={self._offset_z:+.4f} deg/s")
            if abs(self._offset_z + 1.78) > 0.5:
                print(
                    "  WARNING: offset lejos del habitual (-1.78). "
                    "Si el robot se movio durante la calibracion, "
                    "MATAR Y REPETIR: toda la corrida saldra con el rumbo mal."
                )
            return True
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            print("MPU6050 UNAVAILABLE:", self._error)
            self.stop()
            return False

    def _worker(self) -> None:
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                gz = self._read_z() - self._offset_z
                with self._lock:
                    dt = min(max(now - self._last_time, 0.0), 0.05)
                    self._last_time = now
                    self._filtered_z = (
                        MPU_FILTER_ALFA * self._filtered_z
                        + (1.0 - MPU_FILTER_ALFA) * gz
                    )
                    if abs(self._filtered_z) < MPU_ZONE_DEADBAND_DPS:
                        self._filtered_z = 0.0
                    self._heading += self._filtered_z * dt
                self._stop.wait(MPU_PERIOD_S)
            except Exception as exc:
                with self._lock:
                    self._ready = False
                    self._error = f"{type(exc).__name__}: {exc}"
                print("MPU6050 ERROR:", self._error)
                print("WARNING: se continua con los respaldos by tiempo")
                break

    def heading(self) -> float:
        """Code-specific behavior for heading."""
        with self._lock:
            return self._heading

    def heading_total(self) -> float:
        """Code-specific behavior for rumbo."""
        signo = self.signo if self.signo is not None else 1.0
        return signo * self.heading()

    def delta(self, referencia: float) -> float:
        """Code-specific behavior for delta."""
        return self.heading() - referencia

    def oriented_delta(self, referencia: float) -> float:
        """Code-specific behavior for delta orientado."""
        signo = self.signo if self.signo is not None else 1.0
        return signo * (self.heading() - referencia)

    def learn_sign(self, delta_crudo: float, direction: float) -> bool:
        """Code-specific behavior for aprender signo."""
        if self.signo is not None:
            return True
        if abs(delta_crudo) < MPU_SIGN_THRESHOLD_DEG:
            return False
        self.signo = math.copysign(1.0, delta_crudo) * direction
        print(
            f"MPU SIGNO APRENDIDO = {self.signo:+.0f} "
            f"(delta crudo {delta_crudo:+.1f} deg girando {direction:+.0f})"
        )
        return True

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        with self._lock:
            self._ready = False
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
        self._bus = None


mpu = MPU6050()


                                                                             
if AUTOTEST:
    motor = servo = None
else:
    motor = Motor(forward=PIN_MOTOR_IN1, backward=PIN_MOTOR_IN2)
    servo = AngularServo(
        PIN_SERVO,
        min_angle=-90,
        max_angle=90,
        min_pulse_width=0.0005,
        max_pulse_width=0.0025,
        initial_angle=0,
    )

    if not SIM and MOTOR_PWM_HZ:
        try:
            motor.forward_device.pin.frequency = MOTOR_PWM_HZ
            motor.backward_device.pin.frequency = MOTOR_PWM_HZ
        except Exception as exc:
            print("WARNING: no se pudo fijar la frecuencia PWM:", exc)

_ultimo_giro = 0.0
_ultima_direccion = 0                                           


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def ang_signed(a: float) -> float:
    return ((a + 180.0) % 360.0) - 180.0


def set_steering(deg: float, forzar: bool = False) -> None:
    """Code-specific behavior for fijar giro."""
    global _ultimo_giro
    deg = clamp(deg, -LIMIT_STEERING_RIGHT_DEG, LIMIT_STEERING_LEFT_DEG)
    if not forzar and abs(deg - _ultimo_giro) < SERVO_BAND_DEADBAND_DEG:
        return
    _ultimo_giro = deg
    servo.angle = clamp(deg + OFFSET_CENTER, -90.0, 90.0)


def move_forward(v: float) -> None:
    global _ultima_direccion
    _ultima_direccion = 1
    motor.forward(speed=clamp(v, 0.0, 1.0))


def move_reverse(v: float) -> None:
    global _ultima_direccion
    _ultima_direccion = -1
    motor.backward(speed=clamp(v, 0.0, 1.0))


def stop_motion() -> None:
    global _ultima_direccion
    _ultima_direccion = 0
    motor.stop()


def brake() -> None:
    """Code-specific behavior for frenar."""
    direccion = _ultima_direccion
    if direccion == 0:
        motor.stop()
        return
    if direccion > 0:
        motor.backward(speed=BRAKE_SPEED)
    else:
        motor.forward(speed=BRAKE_SPEED)
    time.sleep(BRAKE_PULSE_S)
    stop_motion()


                                                                             
lidar_stop = threading.Event()
lidar_condition = threading.Condition()
lidar_connected = False
lidar_last_time = 0.0
lidar_seq = 0
front_hist: Deque[Tuple[float, float]] = deque(maxlen=HISTORY)
left_hist: Deque[Tuple[float, float]] = deque(maxlen=HISTORY)
right_hist: Deque[Tuple[float, float]] = deque(maxlen=HISTORY)
yaw_left_hist: Deque[Tuple[float, float]] = deque(maxlen=HISTORY)
yaw_right_hist: Deque[Tuple[float, float]] = deque(maxlen=HISTORY)


def front_distance(ds: List[float]) -> Optional[float]:
    """Code-specific behavior for distancia frontal."""
    ds = sorted(d for d in ds if DIST_MIN_MM <= d <= DIST_MAX_MM)
    if len(ds) < SECTOR_MIN_PUNTOS:
        return None
    return float(median(ds[:min(5, len(ds))]))


def wall_distance(ds: List[float]) -> Optional[float]:
    """Code-specific behavior for distancia pared."""
    ds = sorted(d for d in ds if DIST_MIN_MM <= d <= DIST_MAX_MM)
    if len(ds) < SECTOR_MIN_PUNTOS:
        return None
    k = len(ds) // 8
    nucleo = ds[k:len(ds) - k] if len(ds) - 2 * k >= 3 else ds
    return float(median(nucleo))


def wall_geometry(
    perp: Optional[float], fwd: Optional[float]
) -> Tuple[Optional[float], Optional[float]]:
    """Code-specific behavior for geometria pared."""
    if perp is None:
        return None, None
    if fwd is None:
        return perp, None
    theta = math.radians(WALL_THETA_DEG)
    alfa = math.atan2(fwd * math.cos(theta) - perp, fwd * math.sin(theta))
    grados = math.degrees(alfa)
    if abs(grados) > YAW_MAX_VALIDO_DEG:
        return perp, None
    return perp * math.cos(alfa), grados


def recent_median(
    historial: Deque[Tuple[float, float]], now: float
) -> Optional[float]:
    """Code-specific behavior for mediana reciente."""
    valores = [valor for instante, valor in historial
               if now - instante <= SECTOR_MAX_AGE_S]
    return None if not valores else float(median(valores))


def _in_window(a: float, lo: float, hi: float) -> bool:
    return lo <= a <= hi


# Acquire and filter LiDAR scans in a background thread
def lidar_worker() -> None:
    global lidar_connected, lidar_last_time, lidar_seq
    while not lidar_stop.is_set():
        dev = None
        try:
            dev = RPLidar(PORT_NAME, timeout=3)
            dev.start_motor()
            if lidar_stop.wait(0.8):
                break
            with lidar_condition:
                front_hist.clear(); left_hist.clear(); right_hist.clear()
                yaw_left_hist.clear(); yaw_right_hist.clear()
                lidar_connected = True
                lidar_condition.notify_all()

            for raw_scan in dev.iter_scans(max_buf_meas=500, min_len=5):
                if lidar_stop.is_set():
                    break
                fp: List[float] = []
                izq_perp: List[float] = []
                izq_fwd: List[float] = []
                izq_ancho: List[float] = []
                der_perp: List[float] = []
                der_fwd: List[float] = []
                der_ancho: List[float] = []

                for quality, angle, distance in raw_scan:
                    if quality < QUALITY_MIN or distance <= 0:
                        continue
                    a = (float(angle) - LIDAR_OFFSET_DEG) % 360.0
                    if REAR_BLIND_MIN_DEG <= a <= REAR_BLIND_MAX_DEG:
                        continue
                    d = float(distance)
                    if FRONT_MIN_DEG <= ang_signed(a) <= FRONT_MAX_DEG:
                        fp.append(d)
                                                                     
                                                                      
                    if _in_window(a, DER_ANCHO_MIN, DER_ANCHO_MAX):
                        der_ancho.append(d)
                    if _in_window(a, DER_PERP_MIN, DER_PERP_MAX):
                        der_perp.append(d)
                    if _in_window(a, DER_FWD_MIN, DER_FWD_MAX):
                        der_fwd.append(d)
                    if _in_window(a, IZQ_ANCHO_MIN, IZQ_ANCHO_MAX):
                        izq_ancho.append(d)
                    if _in_window(a, IZQ_PERP_MIN, IZQ_PERP_MAX):
                        izq_perp.append(d)
                    if _in_window(a, IZQ_FWD_MIN, IZQ_FWD_MAX):
                        izq_fwd.append(d)

                f = front_distance(fp)
                l_perp = wall_distance(izq_perp)
                r_perp = wall_distance(der_perp)
                if l_perp is None:
                    l_perp = wall_distance(izq_ancho)
                if r_perp is None:
                    r_perp = wall_distance(der_ancho)
                l, yaw_l = wall_geometry(l_perp, wall_distance(izq_fwd))
                r, yaw_r = wall_geometry(r_perp, wall_distance(der_fwd))

                now = time.monotonic()
                with lidar_condition:
                    if f is not None: front_hist.append((now, f))
                    if l is not None: left_hist.append((now, l))
                    if r is not None: right_hist.append((now, r))
                    if yaw_l is not None: yaw_left_hist.append((now, yaw_l))
                    if yaw_r is not None: yaw_right_hist.append((now, yaw_r))
                    lidar_last_time = now
                    lidar_seq += 1
                    lidar_connected = True
                    lidar_condition.notify_all()

        except Exception as exc:
            print("LIDAR REINICIANDO:", type(exc).__name__, exc)
        finally:
            with lidar_condition:
                lidar_connected = False
                lidar_condition.notify_all()
            if dev is not None:
                for fn in (dev.stop, dev.stop_motor, dev.disconnect):
                    try:
                        fn()
                    except Exception:
                        pass
        lidar_stop.wait(LIDAR_RETRY_S)


def read_scan() -> Scan:
    now = time.monotonic()
    with lidar_condition:
        age = float("inf") if lidar_last_time == 0 else now - lidar_last_time
        healthy = lidar_connected and age <= LIDAR_TIMEOUT_S
        f = recent_median(front_hist, now)
        l = recent_median(left_hist, now)
        r = recent_median(right_hist, now)
        yl = recent_median(yaw_left_hist, now)
        yr = recent_median(yaw_right_hist, now)
        seq = lidar_seq
    return Scan(f, l, r, yl, yr, healthy, seq, age)


def wait_for_lidar(timeout: float = 12.0) -> Scan:
    limite = time.monotonic() + timeout
    while time.monotonic() < limite:
        s = read_scan()
        if s.healthy and (s.left is not None or s.right is not None):
            return s
        time.sleep(0.05)
    raise Fatal("LiDAR no disponible")


                                                                             
vision_lock = threading.Lock()
vision_stop = threading.Event()
vision_last_time = 0.0
vision_data = Vision(None, None, 0.0, 0.0, False, float("inf"), 0)


def pillar_shape(w: float, h: float, area: float) -> bool:
    """Code-specific behavior for forma de pilar."""
    if h < VISION_ALTO_MIN_PX or w <= 0.0:
        return False
    if h < VISION_ASPECTO_MIN * w:
        return False
    llenado = area / (w * h)
    return llenado >= VISION_FILL_MIN


def color_blobs(mask) -> List[Tuple[float, float, float, float]]:
    """Code-specific behavior for blobs color."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None, iterations=1)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    candidatos: List[Tuple[float, float, float, float]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < VISION_MIN_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if not pillar_shape(float(w), float(h), area):
            continue
        cx = (x + 0.5 * w) / CAM_W
        bottom = (y + h) / CAM_H
        candidatos.append((area * (0.5 + bottom), cx, bottom, area))
    candidatos.sort(key=lambda c: -c[0])
    return candidatos


def select_pillar(
    rojos: List[Tuple[float, float, float, float]],
    verdes: List[Tuple[float, float, float, float]],
) -> Tuple[Optional[str], Optional[float], float, float]:
    """Code-specific behavior for elegir pilar."""
    todos = [("RED",) + c for c in rojos] + [("GREEN",) + c for c in verdes]
    if not todos:
        return None, None, 0.0, 0.0
    plausibles = [c for c in todos if pillar_is_plausible(c[3], c[4])]
    fuente = plausibles if plausibles else todos
    color, _score, cx, bottom, area = max(fuente, key=lambda c: c[1])
    return color, cx, bottom, area


def distance_from_bottom(bottom: float) -> Optional[float]:
    """Code-specific behavior for distancia by bottom."""
    delta = bottom - PILAR_B_HORIZONTE
    if delta <= 0.0 or bottom < PILAR_B_MINIMUM:
        return None
    return PILAR_K_BOTTOM_MM / delta


def pillar_is_plausible(bottom: float, area: float) -> bool:
    """Code-specific behavior for pilar plausible."""
    if area <= 0.0 or bottom <= 0.0:
        return False
    if bottom >= PILAR_RAZON_BOTTOM_MAX:
                                                                    
        return True
    d_bottom = distance_from_bottom(bottom)
    if d_bottom is None:
        return False
    razon = (PILAR_K_AREA_MM / math.sqrt(area)) / d_bottom
    return PILAR_RAZON_MIN <= razon <= PILAR_RAZON_MAX


def _publish_vision(color, x, bottom, area, seq) -> None:
    global vision_data, vision_last_time
    with vision_lock:
        vision_last_time = time.monotonic()
        vision_data = Vision(color, x, bottom, area, True, 0.0, seq)


# Detect red and green pillars in a background thread
def camera_worker() -> None:
    """Code-specific behavior for camera worker."""
    while not vision_stop.is_set():
        _camera_session()
        if not vision_stop.is_set():
            print("CAMERA REINICIANDO")
            vision_stop.wait(1.0)


def _camera_session() -> None:
    camera = None
    try:
        camera = Picamera2()
        config = camera.create_video_configuration(
            main={"size": (CAM_W, CAM_H), "format": "RGB888"},
            controls={"FrameRate": CAM_FPS},
            buffer_count=4,
            queue=False,
        )
        camera.configure(config)
        camera.start()
        time.sleep(2.0)

                                                                         
        try:
            metadata = camera.capture_metadata()
            controls = {}
            if "ExposureTime" in metadata and "AnalogueGain" in metadata:
                controls.update(
                    AeEnable=False,
                    ExposureTime=metadata["ExposureTime"],
                    AnalogueGain=metadata["AnalogueGain"],
                )
            if "ColourGains" in metadata:
                controls.update(
                    AwbEnable=False, ColourGains=metadata["ColourGains"]
                )
            if controls:
                camera.set_controls(controls)
        except Exception:
            pass

        y0 = int(CAM_H * ROI_PILARES[0])
        y1 = int(CAM_H * ROI_PILARES[1])
        candidate_prev = None
        confirmations = 0
        seq = 0
        while not vision_stop.is_set():
            frame = camera.capture_array("main")
            if SIM:
                color, x, bottom, area = mundo.vision()
                seq += 1
                _publish_vision(color, x, bottom, area, seq)
                time.sleep(1.0 / CAM_FPS)
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            red = cv2.bitwise_or(
                cv2.inRange(hsv, HSV_RED1_LO, HSV_RED1_HI),
                cv2.inRange(hsv, HSV_RED2_LO, HSV_RED2_HI),
            )
            green = cv2.inRange(hsv, HSV_GREEN_LO, HSV_GREEN_HI)
                                                                 
            red[:y0] = 0; red[y1:] = 0
            green[:y0] = 0; green[y1:] = 0

            candidate, x, bottom, area = select_pillar(
                color_blobs(red), color_blobs(green)
            )

            if candidate is not None and candidate == candidate_prev:
                confirmations += 1
            else:
                candidate_prev = candidate
                confirmations = 1 if candidate is not None else 0
            stable = candidate if confirmations >= VISION_CONFIRM_FRAMES else None
            if stable is None:
                x = None; bottom = 0.0; area = 0.0

            seq += 1
            _publish_vision(stable, x, bottom, area, seq)
    except Exception as exc:
        print("ERROR CAMERA:", type(exc).__name__, exc)
    finally:
        if camera is not None:
            try: camera.stop()
            except Exception: pass
            try: camera.close()
            except Exception: pass


def read_vision() -> Vision:
    with vision_lock:
        data = vision_data
        last = vision_last_time
    age = float("inf") if last == 0 else time.monotonic() - last
    if age > CAM_MAX_AGE_S:
        return Vision(None, None, 0.0, 0.0, False, age, data.seq)
    return Vision(
        data.color, data.x, data.bottom, data.area, data.healthy, age, data.seq
    )


def wait_for_camera(timeout: float = 10.0) -> None:
    limite = time.monotonic() + timeout
    while time.monotonic() < limite:
        v = read_vision()
        if v.healthy and v.seq >= 3:
            return
        time.sleep(0.05)
    raise Fatal("Camara no disponible")


                                                                             
def lateral(s: Scan, pared_ref: str) -> Optional[float]:
    """Code-specific behavior for lateral."""
    return s.left if pared_ref == LEFT else s.right


def yaw_pared(s: Scan, pared_ref: str) -> Optional[float]:
    return s.yaw_left if pared_ref == LEFT else s.yaw_right


def steer_toward_wall(pared_ref: str) -> float:
    """Code-specific behavior for giro hacia pared."""
    return 1.0 if pared_ref == LEFT else -1.0


def opposite_wall(pared_ref: str) -> str:
    return RIGHT if pared_ref == LEFT else LEFT


def wall_for_front_recovery(
    s: "Scan", pared_ref: str, carril_activo: bool
) -> Optional[str]:
    """Code-specific behavior for pared para reversa frontal."""
    if carril_activo:
        return opposite_wall(pared_ref)
    izq, der = s.left, s.right
    if izq is None and der is None:
        return None
    if izq is None:
        return RIGHT
    if der is None:
        return LEFT
    return LEFT if izq <= der else RIGHT


# Cascaded wall-distance and heading controller
def straight_control(
    d: Optional[float],
    yaw: Optional[float],
    pared_ref: str,
    d_anterior: Optional[float],
    t_anterior: float,
    now: float,
    rumbo_ref: Optional[float] = None,
    integral: float = 0.0,
    objetivo: float = DISTANCE_LATERAL_TARGET_MM,
    giro_max: float = STEERING_CONTROL_MAX_DEG,
    carril: bool = False,
    permitir_ciega: bool = True,
) -> Tuple[float, float]:
    """Code-specific behavior for control recta."""
    hacia = steer_toward_wall(pared_ref)

                                                                            
    if carril and rumbo_ref is not None and mpu.ready:
        if d is not None and d <= PILAR_DISTANCE_CRITICA_MM:
            return -hacia * STEERING_ESCAPE_DEG, SPEED_ESCAPE
        if d is None or d > PILAR_SIGNAL_MAX_MM:
            if not permitir_ciega:
                                                                         
                                                                    
                return 0.0, PILAR_EXIT_SPEED
            offset = hacia * PILAR_HEADING_OFFSET_MAX_DEG                  
        else:
            offset = clamp(                                                 
                hacia * PILAR_KP_DIST_A_HEADING * (d - objetivo),
                -PILAR_HEADING_OFFSET_MAX_DEG,
                PILAR_HEADING_OFFSET_MAX_DEG,
            )
        error_rumbo = (rumbo_ref + offset) - mpu.heading_total()
        return (
            clamp(PILAR_KA_HEADING * error_rumbo, -giro_max, giro_max),
            PILAR_SPEED,
        )

                                                                            
    if d is None:
                                                                         
                                                                        
         
                                                                             
                                                                         
                                                                          
                                                                            
                                                                           
                                                                        
        if rumbo_ref is not None and mpu.ready:
            return (
                clamp(
                    KA_HEADING * (rumbo_ref - mpu.heading_total()),
                    -giro_max,
                    giro_max,
                ),
                SPEED_APPROACH,
            )
        return 0.0, SPEED_APPROACH

    if d <= DISTANCE_ESCAPE_MM:
        return -hacia * STEERING_ESCAPE_DEG, SPEED_ESCAPE

    if rumbo_ref is not None and mpu.ready:
        offset = clamp(
            hacia * KP_DIST_A_HEADING * (d - objetivo),
            -HEADING_OFFSET_MAX_DEG,
            HEADING_OFFSET_MAX_DEG,
        )
        error_rumbo = (rumbo_ref + offset + integral) - mpu.heading_total()
        cmd = KA_HEADING * error_rumbo
    else:
                                                                              
        tasa = 0.0
        if d_anterior is not None:
            dt = now - t_anterior
            if 0.0 < dt <= DERIVADA_MAX_DT_S:
                tasa = clamp((d - d_anterior) / max(dt, 0.03), -350.0, 350.0)
        termino_distancia = KP_LATERAL * (d - objetivo)
        termino_yaw = KA_YAW * yaw if yaw is not None else 0.0
        cmd = hacia * (termino_distancia + termino_yaw + KD_LATERAL * tasa)
    return clamp(cmd, -giro_max, giro_max), SPEED_STRAIGHT


def pillar_command(
    color: str,
    bottom: float,
    front: Optional[float],
    x: Optional[float] = None,
) -> float:
    """Code-specific behavior for comando pilar."""
    progreso_vision = clamp(
        (bottom - PILAR_BOTTOM_INICIO)
        / (PILAR_BOTTOM_MAX - PILAR_BOTTOM_INICIO),
        0.0,
        1.0,
    )
    progreso_lidar = 0.0
    if front is not None:
        progreso_lidar = clamp(
            (PILAR_DIST_INICIO_MM - front)
            / (PILAR_DIST_INICIO_MM - PILAR_DIST_REFERENCE_MM),
            0.0,
            1.0,
        )
    magnitud = PILAR_STEERING_MIN_DEG + max(progreso_vision, progreso_lidar) * (
        PILAR_STEERING_MAX_DEG - PILAR_STEERING_MIN_DEG
    )
    if color == "GREEN":
        magnitud += PILAR_STEERING_EXTRA_GREEN_DEG
    cmd = -magnitud if color == "RED" else magnitud

    if x is not None:
        objetivo = (
            PILAR_X_TARGET_RED if color == "RED" else PILAR_X_TARGET_GREEN
        )
        cmd += clamp(
            PILAR_KP_X * (objetivo - x),
            -PILAR_ADJUSTMENT_MAX_DEG,
            PILAR_ADJUSTMENT_MAX_DEG,
        )
    return clamp(cmd, -LIMIT_STEERING_RIGHT_DEG, LIMIT_STEERING_LEFT_DEG)


def rotation_compensated_x(x: float, rel_deg: Optional[float]) -> float:
    """Code-specific behavior for x sin rotacion."""
    if rel_deg is None:
        return x
    ang = math.degrees(math.atan((x - 0.5) * CAM_W / CAMARA_FOCAL_PX)) - rel_deg
    ang = clamp(ang, -80.0, 80.0)
    return 0.5 + math.tan(math.radians(ang)) * CAMARA_FOCAL_PX / CAM_W


def pillar_passed(
    color: Optional[str], x: Optional[float], rel_deg: Optional[float] = None
) -> bool:
    """Code-specific behavior for pilar rebasado."""
    if x is None or color is None:
        return False
    xc = rotation_compensated_x(x, rel_deg)
    if color == "RED":
        return xc <= PILAR_X_PASSED_RED
    return xc >= PILAR_X_PASSED_GREEN


def pillar_hold_time(color: Optional[str]) -> float:
    return PILAR_HOLD_GREEN_S if color == "GREEN" else PILAR_HOLD_RED_S


def front_limit(pilar_activo: bool, v: Vision, color: Optional[str]) -> float:
    """Code-specific behavior for limite frontal."""
    if not pilar_activo or v.color != color or v.x is None:
        return FRONT_CRITICAL_MM
    centrado = abs(v.x - 0.5) <= PILAR_FRONT_X_MAX
    cerca = v.bottom >= PILAR_FRONT_BOTTOM_MIN
    return PILAR_DIST_STOP_MM if (centrado and cerca) else FRONT_CRITICAL_MM


def countersteer_command(color: str, oriented_delta: Optional[float]) -> float:
    """Code-specific behavior for comando contragiro."""
    if oriented_delta is None:
        return COUNTERSTEER_DEG if color == "RED" else -COUNTERSTEER_DEG
    magnitud = (
        MPU_COUNTERSTEER_LENTO_DEG
        if abs(oriented_delta) <= MPU_COUNTERSTEER_LENTO_THRESHOLD_DEG
        else MPU_COUNTERSTEER_MAX_DEG
    )
    if mpu.signo is None:
        return magnitud if color == "RED" else -magnitud
    return -math.copysign(magnitud, oriented_delta)


                                                                             
def impossible_jump(
    d: Optional[float], d_previo: Optional[float], dt: float
) -> bool:
    """Code-specific behavior for salto imposible."""
    if d is None or d_previo is None or dt <= 0.0:
        return False
    return abs(d - d_previo) / dt > JUMP_MAX_MM_POR_S


# Reverse safely and retry after a recoverable condition
def recovery_maneuver(reason: str, wall: Optional[str] = None) -> None:
    """Code-specific behavior for maniobra recuperacion."""
    global _intentos_seguidos
    print(f"FSM RECUPERAR: {reason}")
    brake()
    set_steering(0, forzar=True)
    time.sleep(RECOVER_PAUSA_S)

    if wall is not None:
                                                                            
        angulo = steer_toward_wall(wall) * RECOVER_ANGLE_DEG
        print(f"FSM RECUPERAR: reversa girada {angulo:+.0f} deg (pared {wall})")
        set_steering(angulo, forzar=True)
        time.sleep(WAIT_SERVO_REVERSE_S)

    limite = RECOVER_REVERSE_MAX_S + _intentos_seguidos * RECOVER_EXTRA_POR_INTENTO_S
    ritmo = FixedRateLoop(CONTROL_PERIOD_S)
    inicio = time.monotonic()
    move_reverse(RECOVER_REVERSE_SPEED)
    while time.monotonic() - inicio < limite:
        ritmo.wait()
        s = read_scan()
        if s.healthy and s.front is not None and s.front >= RECOVER_FRONT_LIBRE_MM:
            break
    brake()
    set_steering(0, forzar=True)
    time.sleep(RECOVER_PAUSA_S)
    wait_for_lidar(6.0)
    print("FSM RECUPERAR: listo para reintentar")


def run_state(nombre: str, funcion, *args, **kwargs):
    """Code-specific behavior for ejecutar."""
    global recoveries, _intentos_seguidos
    _intentos_seguidos = 0
    while True:
        try:
            resultado = funcion(*args, **kwargs)
            _intentos_seguidos = 0
            return resultado
        except RecoverableError as exc:
            recoveries += 1
            _intentos_seguidos += 1
            print(
                f"INCIDENTE {recoveries}/{MAX_RECUPERACIONES} "
                f"en {nombre}: {exc}"
            )
            if recoveries > MAX_RECUPERACIONES:
                raise Fatal(f"demasiados incidentes en {nombre}: {exc}")
            recovery_maneuver(str(exc), getattr(exc, "pared", None))


recoveries = 0
_intentos_seguidos = 0


                                                                             
SENTIDO_ABIERTO_MM = 1200.0                                               


def determine_outer_wall(
    max_izq: float, max_der: float,
    izq: Optional[float], der: Optional[float],
) -> str:
    """Code-specific behavior for decidir pared outer."""
    abierto_izq = max_izq >= SENTIDO_ABIERTO_MM
    abierto_der = max_der >= SENTIDO_ABIERTO_MM
    if abierto_izq != abierto_der:
        return RIGHT if abierto_izq else LEFT
    if izq is None and der is not None:
        return RIGHT
    if der is None and izq is not None:
        return LEFT
    if izq is not None and der is not None:
        return RIGHT if izq > der else LEFT
    return RIGHT if max_izq >= max_der else LEFT


def corridor_centering(s: "Scan") -> float:
    """Code-specific behavior for centrado corredor."""
    izq, der = s.left, s.right
    if izq is None or der is None:
        return 0.0
    if izq > WALL_MAX_VALIDA_MM or der > WALL_MAX_VALIDA_MM:
        return 0.0
    return clamp(
        CENTRADO_KP_DEG_POR_MM * (izq - der),
        -CENTRADO_MAX_DEG, CENTRADO_MAX_DEG,
    )


# Approach the first corner and identify the outer wall
def determine_direction() -> str:
    """Code-specific behavior for detectar sentido."""
    wait_for_lidar()
    ritmo = FixedRateLoop(CONTROL_PERIOD_S)
    confirm = 0
    pilar_color: Optional[str] = None
    pilar_last_seen = 0.0
    pilar_activo_anterior = False
    pilar_rumbo_ref: Optional[float] = None
    recuperando = False
    recuperacion_hasta = 0.0
    recuperacion_ok = 0
    last_debug = 0.0
                                                                         
                                                                           
                          
     
                                                                            
                                                                     
                                                                          
                                                                          
                                                                      
                                                                       
    pilar_bloqueado_hasta = 0.0
    frente_despejado_visto = False
                                                                      
                                                                          
    max_izq = 0.0
    max_der = 0.0
    move_forward(SPEED_APPROACH)
    print("FSM BUSCAR_SENTIDO")

    while True:
        now = ritmo.wait()
        s = read_scan()
        if not s.healthy:
            stop_motion()
            print("FSM RECUPERAR_LIDAR: robot detenido")
            wait_for_lidar(12.0)
            move_forward(SPEED_APPROACH)
            continue

        if s.left is not None:
            max_izq = max(max_izq, s.left)
        if s.right is not None:
            max_der = max(max_der, s.right)

        v = read_vision()
        if (
            v.healthy
            and v.color in ("RED", "GREEN")
            and v.bottom >= PILAR_ACTIVATION_BOTTOM
            and v.area >= PILAR_ACTIVATION_AREA_PX
            and pillar_is_plausible(v.bottom, v.area)
        ):
            pilar_color = v.color
            pilar_last_seen = now
        pilar_activo = (
            pilar_color is not None
            and now - pilar_last_seen <= pillar_hold_time(pilar_color)
            and not pillar_passed(
                pilar_color, v.x if v.color == pilar_color else None,
                mpu.oriented_delta(pilar_rumbo_ref)
                if mpu.ready and pilar_rumbo_ref is not None else None,
            )
        )

        if pilar_activo:
                                                                            
                                                                   
            frente_despejado_visto = False
        if pilar_activo and not pilar_activo_anterior:
            pilar_rumbo_ref = mpu.heading() if mpu.ready else None
            recuperando = False
        elif not pilar_activo and pilar_activo_anterior:
            pilar_bloqueado_hasta = now + PILAR_REFRACTORY_S
            delta = (
                mpu.oriented_delta(pilar_rumbo_ref)
                if mpu.ready and pilar_rumbo_ref is not None
                else None
            )
            recuperando = (
                delta is not None and abs(delta) >= MPU_DELTA_MIN_EVASION_DEG
            )
            if recuperando:
                recuperacion_hasta = now + MPU_RECOVERY_TIMEOUT_S
                recuperacion_ok = 0
                print(f"FSM CONTRAGIRO INICIAL delta={delta:+.2f} deg")
        pilar_activo_anterior = pilar_activo

        limite_f = front_limit(pilar_activo, v, pilar_color)
        if s.front is not None and s.front <= limite_f:
            brake(); set_steering(0, forzar=True)
            raise RecoverableError(
                f"Frente critico en BUSCAR_SENTIDO: {s.front:.0f} mm "
                f"(limite {limite_f:.0f})"
            )

        delta = (
            mpu.oriented_delta(pilar_rumbo_ref)
            if recuperando and mpu.ready and pilar_rumbo_ref is not None
            else None
        )
        if recuperando:
            if delta is not None and abs(delta) <= MPU_TOLERANCE_HEADING_DEG:
                recuperacion_ok += 1
            else:
                recuperacion_ok = 0
            if (
                recuperacion_ok >= MPU_HEADING_CONFIRMATIONS
                or now >= recuperacion_hasta
                or delta is None
            ):
                recuperando = False
                print("MPU RUMBO INICIAL RECUPERADO")

        if pilar_activo:
            mismo = v.color == pilar_color
            cmd = pillar_command(
                pilar_color,
                v.bottom if mismo else PILAR_BOTTOM_INICIO,
                s.front,
                v.x if mismo else None,
            )
            vel = PILAR_SPEED
        elif recuperando:
            cmd = countersteer_command(pilar_color, delta)
            vel = RECOVERY_SPEED
        else:
            cmd = corridor_centering(s)
            vel = SPEED_APPROACH

        set_steering(cmd)
        move_forward(vel)

        if s.front is None or s.front > DISTANCE_DESPEJADO_MM:
            frente_despejado_visto = True

        pilar_cerca = pilar_activo or now < pilar_bloqueado_hasta
        if (
            not pilar_cerca
            and not recuperando
            and frente_despejado_visto
            and s.front is not None
        ):
            confirm = confirm + 1 if s.front <= DISTANCE_INICIAR_STEERING_MM else 0
        else:
            confirm = 0

        tele.write_row(
            estado=State.DETERMINE_DIRECTION.name, corner=0,
            d_izq="" if s.left is None else round(s.left),
            d_der="" if s.right is None else round(s.right),
            front="" if s.front is None else round(s.front),
            cmd=round(cmd, 1), vel=round(vel, 2),
            pilar=pilar_color if pilar_activo else "",
            pilar_x="" if v.x is None else round(v.x, 2),
            pilar_b=round(v.bottom, 2),
            heading_total=round(mpu.heading_total(), 1) if mpu.ready else "",
            lidar_ok=int(s.healthy), vision_ok=int(v.healthy),
        )

        if DEBUG and now - last_debug >= 0.20:
            last_debug = now
            print(
                f"SENTIDO F={'---' if s.front is None else f'{s.front:.0f}'} "
                f"I={'---' if s.left is None else f'{s.left:.0f}'} "
                f"D={'---' if s.right is None else f'{s.right:.0f}'} "
                f"st={cmd:+.1f} confirm={confirm}/{CONFIRMATIONS_CORNER}"
            )

        if confirm >= CONFIRMATIONS_CORNER:
            brake()
            wall = determine_outer_wall(max_izq, max_der, s.left, s.right)
            direction = "clockwise" if wall == LEFT else "anticlockwise"
            print(
                f"SENTIDO DEFINIDO: {direction}, pared exterior {wall} "
                f"(I={'---' if s.left is None else f'{s.left:.0f}'} "
                f"D={'---' if s.right is None else f'{s.right:.0f}'} | "
                f"max I={max_izq:.0f} D={max_der:.0f})"
            )
            for s_, color, ref, etiqueta in rules_table():
                if s_ == direction:
                    print(f"  pilar {color:5s} -> pared {ref} ({etiqueta})")
            return wall


                                                                             
# Follow the reference wall and avoid pillars until a corner
def drive_section(
    outer_wall: str, corner: int, limite_s: Optional[float] = None
) -> str:
    """Code-specific behavior for recorrer tramo."""
    print(f"FSM RECTO | pared exterior {outer_wall}")
    ritmo = FixedRateLoop(CONTROL_PERIOD_S)
    inicio = time.monotonic()
    confirm = 0
    d_prev: Optional[float] = None
    t_prev = inicio
    seq_prev = -1
    last_debug = 0.0
    last_aviso_vision = 0.0

    pilar_color: Optional[str] = None
    pilar_last_seen = 0.0
    pilar_activo_anterior = False
    pilar_rumbo_ref: Optional[float] = None
    pilar_bloqueado_hasta = 0.0
    pilar_descartado = None

    critica_confirm = 0
    pared_perdida_desde: Optional[float] = None
    d_valida: Optional[float] = None
    t_valida = inicio
    saltos_seguidos = 0
    scan_rechazado = False
    rumbo_ref = mpu.heading_total() if mpu.ready else None
    frente_despejado_visto = False
    integral = 0.0

                                                                         
    carril_color: Optional[str] = None
    salida_anterior = False
    pared_ref = outer_wall
    objetivo_lateral = DISTANCE_LATERAL_TARGET_MM

    while True:
        now = ritmo.wait()
        s = read_scan()
        if not s.healthy:
            stop_motion()
            print("FSM RECUPERAR_LIDAR: robot detenido")
            wait_for_lidar(12.0)
            inicio = time.monotonic()
            ritmo.reset()
            d_prev = None
            critica_confirm = 0
            continue

                                                                            
        v = read_vision()
        if not v.healthy:
            vel_max_vision = SPEED_APPROACH
            if now - last_aviso_vision >= CAM_AVISO_PERIOD_S:
                last_aviso_vision = now
                print(f"AVISO: VISION CAIDA age={v.age:.2f}s - avance limitado")
        else:
            vel_max_vision = 1.0

        pilar_umbrales = (
            now >= pilar_bloqueado_hasta
            and v.healthy
            and v.color in ("RED", "GREEN")
            and v.bottom >= PILAR_ACTIVATION_BOTTOM
            and v.area >= PILAR_ACTIVATION_AREA_PX
        )
                                                                        
                                                                  
         
                                                                             
                                                                           
                                                                         
                                                                          
                                                                   
        ya_activo = pilar_activo_anterior and v.color == pilar_color
        pilar_valido = pilar_umbrales and (
            ya_activo or pillar_is_plausible(v.bottom, v.area)
        )
        if pilar_umbrales and not pilar_valido:
                                                                
            if v.color != pilar_descartado:
                pilar_descartado = v.color
                d_a = PILAR_K_AREA_MM / math.sqrt(max(v.area, 1.0))
                d_b = distance_from_bottom(v.bottom)
                print(
                    f"PILAR {v.color} DESCARTADO (no es un pilar): "
                    f"b={v.bottom:.2f} area={v.area:.0f} -> "
                    f"d_area={d_a:.0f} "
                    + ("d_bottom=--- (en el horizonte)" if d_b is None
                       else f"d_bottom={d_b:.0f} razon={d_a / d_b:.2f}")
                )
        elif not pilar_umbrales:
            pilar_descartado = None

        if pilar_valido:
            if v.color != pilar_color:
                print(
                    f"PILAR {v.color} detectado x={v.x:.2f} "
                    f"b={v.bottom:.2f} area={v.area:.0f}"
                )
            if not pilar_activo_anterior:
                pilar_rumbo_ref = mpu.heading() if mpu.ready else None
            pilar_color = v.color
            pilar_last_seen = now

        x_actual = v.x if v.color == pilar_color else None
        rel_pilar = (
            mpu.oriented_delta(pilar_rumbo_ref)
            if mpu.ready and pilar_rumbo_ref is not None
            else None
        )
        pilar_activo = (
            pilar_color is not None
            and now - pilar_last_seen <= pillar_hold_time(pilar_color)
            and not pillar_passed(pilar_color, x_actual, rel_pilar)
        )

                                                                            
                                                                          
                       
        if pilar_activo_anterior and not pilar_activo:
            pilar_bloqueado_hasta = now + PILAR_REFRACTORY_S
        pilar_activo_anterior = pilar_activo

                                                                            
                                                                              
                                                                               
                                                                            
                                                                     
        carril_deseado = (
            pilar_color if (pilar_activo or now < pilar_bloqueado_hasta) else None
        )
        if carril_deseado != carril_color:
            carril_color = carril_deseado
            pared_ref_nueva = reference_wall(carril_color, outer_wall)
            if pared_ref_nueva != pared_ref:
                                                                              
                                                                       
                                                                            
                d_prev = None
                integral = 0.0
                pared_perdida_desde = None
                critica_confirm = 0
                d_valida = None
                saltos_seguidos = 0
                etiqueta = (
                    "outer" if pared_ref_nueva == outer_wall else "inner"
                )
                print(
                    f"REFERENCIA -> pared {pared_ref_nueva} ({etiqueta})"
                    + (f" por pilar {carril_color}" if carril_color else " (recta)")
                )
                pared_ref = pared_ref_nueva
            objetivo_lateral = (
                PILAR_TARGET_MM if carril_color else DISTANCE_LATERAL_TARGET_MM
            )
            if carril_color is None:
                print(f"PILAR ATRAS: vuelve a recta normal, objetivo "
                      f"{objetivo_lateral:.0f} mm")
        carril_activo = carril_color is not None
                                                                           
        salida_activa = carril_activo and not pilar_activo
        if salida_activa and not salida_anterior:
            print("SALIDA DE PILAR: termina el cruce sin fase ciega")
        salida_anterior = salida_activa

                                                                            
        d = lateral(s, pared_ref)
        yaw = yaw_pared(s, pared_ref)
        if d is not None and d > WALL_MAX_VALIDA_MM:
            d = None
            yaw = None

                                                                             
                                                                              
                                                               
                                                                          
                                                                          
                                                                         
         
                                                                            
                                                                      
                                                                            
                                                                            
                                                               
                                                                        
                                                          
                                                          
                                                                       
        if s.seq != seq_prev:
            scan_rechazado = impossible_jump(d, d_valida, now - t_valida)
            if scan_rechazado:
                saltos_seguidos += 1
                if saltos_seguidos <= JUMP_CICLOS_MAX:
                    print(
                        f"LIDAR: salto imposible {d_valida:.0f}->{d:.0f} mm "
                        f"en {now - t_valida:.3f}s, descartado"
                    )
                else:
                    saltos_seguidos = 0                                         
                    scan_rechazado = False
            else:
                saltos_seguidos = 0
        if scan_rechazado:
            d = d_valida
            yaw = None
        elif d is not None:
            d_valida = d
            t_valida = now

                                                                          
                                                         
        if d is not None:
            pared_perdida_desde = None
        else:
            if pared_perdida_desde is None:
                pared_perdida_desde = now
            else:
                                                                      
                                                                          
                                                           
                limite_ciego = (
                    CARRIL_BLIND_TIMEOUT_S if carril_activo
                    else WALL_LOST_TIMEOUT_S
                )
                if now - pared_perdida_desde >= limite_ciego:
                    brake(); set_steering(0, forzar=True)
                    raise RecoverableError(
                        "Sin pared en el cruce demasiado tiempo"
                        if carril_activo
                        else "Pared de referencia perdida demasiado tiempo",
                        pared_ref,
                    )

        limite_critico = (
            PILAR_DISTANCE_CRITICA_MM if carril_activo else DISTANCE_CRITICA_MM
        )
        if d is not None and d <= limite_critico:
            critica_confirm += 1
        else:
            critica_confirm = 0
        if critica_confirm >= DISTANCE_CRITICA_CONFIRMATIONS:
            brake()
            set_steering(-steer_toward_wall(pared_ref) * STEERING_ESCAPE_DEG, True)
            raise RecoverableError(f"Pared de referencia critica: {d:.0f} mm", pared_ref)

                                                                            
        if (
            not carril_activo
            and d is not None
            and rumbo_ref is not None
            and mpu.ready
        ):
            integral = clamp(
                integral
                + steer_toward_wall(pared_ref) * KI_DIST_A_HEADING
                * (d - objetivo_lateral) * CONTROL_PERIOD_S,
                -HEADING_INTEGRAL_MAX_DEG, HEADING_INTEGRAL_MAX_DEG,
            )

        cmd, vel = straight_control(
            d, yaw, pared_ref, d_prev, t_prev, now, rumbo_ref, integral,
            objetivo_lateral,
            PILAR_STEERING_CASCADA_MAX_DEG if carril_activo else STEERING_CONTROL_MAX_DEG,
            carril_activo,
            pilar_activo,                                               
        )
        if s.seq != seq_prev:
            seq_prev = s.seq
            d_prev = d
            if d is not None:
                t_prev = now

                                                                            
        if pilar_activo and (rumbo_ref is None or not mpu.ready):
            cmd = pillar_command(
                pilar_color,
                v.bottom if v.color == pilar_color else PILAR_BOTTOM_INICIO,
                s.front,
                x_actual,
            )
            vel = PILAR_SPEED

                                                                      
                                                                         
        if carril_activo and d is not None and d <= PILAR_GAP_MINIMUM_MM:
            hacia = steer_toward_wall(pared_ref)
            if cmd * hacia > 0:
                cmd = hacia * min(abs(cmd), PILAR_STEERING_LIMITADO_DEG)
                print(f"PILAR: hueco {d:.0f} mm, giro limitado a {cmd:+.1f}")

        limite_f = front_limit(pilar_activo, v, pilar_color)
        if s.front is not None and s.front <= limite_f:
            brake(); set_steering(0, forzar=True)
            raise RecoverableError(
                f"Frente critico: {s.front:.0f} mm (limite {limite_f:.0f})",
                wall_for_front_recovery(s, pared_ref, carril_activo),
            )

                                                                            
                                                                            
                                                                    
        limite_escape = (
            PILAR_GAP_MINIMUM_MM if carril_activo else DISTANCE_ESCAPE_MM
        )
        if not pilar_activo and d is not None and d <= limite_escape:
            cmd = -steer_toward_wall(pared_ref) * STEERING_ESCAPE_DEG
            vel = min(vel, SPEED_ESCAPE)

                                                                            
        if s.front is None or s.front <= DISTANCE_FRENAR_MM:
            vel = min(vel, SPEED_APPROACH)
        vel = min(vel, vel_max_vision)

        cerca_esquina = (
            s.front is not None and s.front <= DISTANCE_INICIAR_STEERING_MM
        )
                                                                            
                                                              
        if s.front is None or s.front > DISTANCE_DESPEJADO_MM:
            frente_despejado_visto = True

                                                                             
                                                                       
                                                                      
        pilar_cerca = pilar_activo or now < pilar_bloqueado_hasta
        if not pilar_cerca and (
            frente_despejado_visto or now - inicio >= TIME_MIN_SECTION_S
        ):
            if s.front is not None:
                confirm = confirm + 1 if cerca_esquina else 0
        else:
            confirm = 0

        set_steering(cmd)
        move_forward(vel)

        rumbo_rel = (
            mpu.oriented_delta(pilar_rumbo_ref)
            if mpu.ready and pilar_rumbo_ref is not None
            else None
        )
        tele.write_row(
            estado=State.PILLAR.name if carril_activo else State.STRAIGHT.name,
            corner=corner,
            d="" if d is None else round(d),
            ref=pared_ref[:3],
            obj=round(objetivo_lateral),
            d_izq="" if s.left is None else round(s.left),
            d_der="" if s.right is None else round(s.right),
            front="" if s.front is None else round(s.front),
            yaw="" if yaw is None else round(yaw, 1),
            cmd=round(cmd, 1), vel=round(vel, 2),
            pilar=("" if carril_color is None
                   else carril_color + ("*" if salida_activa else "")),
            pilar_x="" if x_actual is None else round(x_actual, 2),
            pilar_b="" if v.color != pilar_color else round(v.bottom, 2),
            heading_total=round(mpu.heading_total(), 1) if mpu.ready else "",
            rumbo_rel="" if rumbo_rel is None else round(rumbo_rel, 1),
            lidar_ok=int(s.healthy), vision_ok=int(v.healthy),
        )

        if DEBUG and now - last_debug >= 0.20:
            last_debug = now
            print(
                f"RECTO d={'---' if d is None else f'{d:.0f}'}"
                f"/{objetivo_lateral:.0f} ref={pared_ref[:3]} "
                f"F={'---' if s.front is None else f'{s.front:.0f}'} "
                f"st={cmd:+.1f} v={vel:.2f} "
                f"pilar={carril_color or '---'}{'*' if salida_activa else ''} "
                f"rel={'---' if rumbo_rel is None else f'{rumbo_rel:+.0f}'} "
                f"esquina={confirm}/{CONFIRMATIONS_CORNER}"
            )

        if confirm >= CONFIRMATIONS_CORNER:
            brake(); set_steering(0, forzar=True)
            print(f"FSM APROXIMAR COMPLETA F={s.front:.0f} mm")
            return "ESQUINA"

        if limite_s is not None and now - inicio >= limite_s:
            brake(); set_steering(0, forzar=True)
            print("FSM TRAMO FINAL COMPLETO")
            return "TIEMPO"


                                                                             
# Execute reverse, 90-degree turn, and corner exit phases
def corner_maneuver(outer_wall: str, corner: int) -> None:
    """Code-specific behavior for maniobra esquina."""
                                                                       
    signo_giro = -1.0 if outer_wall == LEFT else 1.0
    rumbo_inicial = mpu.heading() if mpu.ready else None

    for attempt in range(1, MAX_ATTEMPTS_CORNER + 1):
        try:
            _reverse_phase(signo_giro, corner)
            _turn_phase(outer_wall, signo_giro, rumbo_inicial, corner)
            _exit_phase(outer_wall, corner)
            return
        except RecoverableError as exc:
            if attempt >= MAX_ATTEMPTS_CORNER:
                raise
            print(f"ESQUINA intento {attempt}/{MAX_ATTEMPTS_CORNER}: {exc}")
            recovery_maneuver(str(exc), getattr(exc, "pared", None))
                                                                              
                                                                           


def _reverse_phase(signo_giro: float, corner: int) -> None:
    """Code-specific behavior for  fase reversa."""
    print("FSM REVERSA")
    brake()
    set_steering(0, forzar=True)
    time.sleep(STOP_ANTES_DIRECTION_S)
    set_steering(-signo_giro * REVERSE_ANGLE_DEG, forzar=True)
    time.sleep(WAIT_SERVO_REVERSE_S)

    ritmo = FixedRateLoop(CONTROL_PERIOD_S)
    inicio = time.monotonic()
    move_reverse(REVERSE_SPEED)
    while True:
        now = ritmo.wait()
        elapsed = now - inicio
        s = read_scan()
        tele.write_row(
            estado=State.REVERSE.name, corner=corner,
            front="" if s.front is None else round(s.front),
            vel=-REVERSE_SPEED,
            heading_total=round(mpu.heading_total(), 1) if mpu.ready else "",
            lidar_ok=int(s.healthy),
        )
        if elapsed >= REVERSE_TIME_MAX_S:
            break
        if (
            elapsed >= REVERSE_TIME_MIN_S
            and s.healthy
            and s.front is not None
            and s.front >= REVERSE_FRONT_TARGET_MM
        ):
            print(f"REVERSA completa F={s.front:.0f} mm t={elapsed:.2f}s")
            break
    brake()
    set_steering(0, forzar=True)
    time.sleep(STOP_CAMBIO_GEAR_S)


def _turn_phase(
    outer_wall: str,
    signo_giro: float,
    rumbo_inicial: Optional[float],
    corner: int,
) -> None:
    print("FSM GIRAR")
    set_steering(signo_giro * STEERING_ANGLE_DEG, forzar=True)
    time.sleep(WAIT_SERVO_REVERSE_S)

    ritmo = FixedRateLoop(CONTROL_PERIOD_S)
    inicio = time.monotonic()
    move_forward(STEERING_SPEED)

    prev: Optional[float] = None
    lateral_inicial: Optional[float] = None
    lateral_perdida = False
    transicion_vista = False
    estables = 0
    ultimo_frente: Optional[float] = None
    last_debug = 0.0
    confirmed = False
    reason = ""

    while True:
        now = ritmo.wait()
        elapsed = now - inicio
        s = read_scan()

                                                                          
        oriented_delta = None
        if mpu.ready and rumbo_inicial is not None:
            crudo = mpu.delta(rumbo_inicial)
            mpu.learn_sign(crudo, signo_giro)
            oriented_delta = mpu.oriented_delta(rumbo_inicial)
            avance_giro = signo_giro * oriented_delta
            if avance_giro >= STEERING_SUAVIZAR_DESDE_DEG:
                set_steering(signo_giro * STEERING_ANGLE_FINAL_DEG)
            if avance_giro >= STEERING_TARGET_DEG and elapsed >= STEERING_TIME_MIN_S:
                confirmed = True
                reason = f"rumbo {avance_giro:.1f} deg"

                                                                          
        if s.healthy:
            d = lateral(s, outer_wall)
            ultimo_frente = s.front
            if s.front is not None and s.front <= STEERING_FRONT_STOP_MM:
                brake(); set_steering(0, forzar=True)
                raise RecoverableError(f"Frente cerrado en giro: {s.front:.0f} mm")
            if lateral_inicial is None and d is not None:
                lateral_inicial = d
            if d is None and elapsed >= WALL_LOST_MIN_S:
                lateral_perdida = True
            if d is not None and (
                lateral_perdida
                or (
                    lateral_inicial is not None
                    and abs(d - lateral_inicial) >= WALL_NEW_JUMP_MM
                )
            ):
                transicion_vista = True
            valido = (
                transicion_vista
                and d is not None
                and WALL_NEW_MIN_MM <= d <= WALL_NEW_MAX_MM
                and prev is not None
                and abs(d - prev) <= WALL_NEW_STABLE_DELTA_MM
            )
            estables = estables + 1 if valido else 0
            if d is not None:
                prev = d
            frente_libre = s.front is None or s.front >= STEERING_FRONT_LIBRE_MM
            if (
                not confirmed
                and elapsed >= STEERING_TIME_MIN_S
                and estables >= WALL_NEW_CONFIRMATIONS
                and frente_libre
                and oriented_delta is None
            ):
                                                                               
                confirmed = True
                reason = "new stable wall"

            if DEBUG and now - last_debug >= 0.15:
                last_debug = now
                print(
                    f"GIRAR t={elapsed:.2f} "
                    f"d={'---' if d is None else f'{d:.0f}'} "
                    f"F={'---' if s.front is None else f'{s.front:.0f}'} "
                    f"delta={'---' if oriented_delta is None else f'{signo_giro*oriented_delta:.1f}'} "
                    f"estable={estables}/{WALL_NEW_CONFIRMATIONS}"
                )

        tele.write_row(
            estado=State.TURN.name, corner=corner,
            ref=outer_wall[:3],
            d_izq="" if s.left is None else round(s.left),
            d_der="" if s.right is None else round(s.right),
            front="" if s.front is None else round(s.front),
            vel=STEERING_SPEED,
            heading_total=round(mpu.heading_total(), 1) if mpu.ready else "",
            lidar_ok=int(s.healthy),
        )

        if confirmed:
            break
        if elapsed >= STEERING_TIME_MAX_S:
            break

    brake()

    if not confirmed:
        set_steering(0, forzar=True)
        frente_txt = "no return" if ultimo_frente is None else f"{ultimo_frente:.0f} mm"
        raise RecoverableError(f"Giro sin confirmar; frente={frente_txt}")
    print(f"FSM GIRO CONFIRMADO por {reason}")


def _exit_phase(outer_wall: str, corner: int) -> None:
    print("FSM SALIR")
    s = read_scan()
    d_salida = lateral(s, outer_wall)
    if d_salida is not None and d_salida < EXIT_CERCA_WALL_MM:
        steer = -steer_toward_wall(outer_wall) * EXIT_ESCAPE_DEG
        print(f"SALIR separando d={d_salida:.0f} mm st={steer:+.1f}")
        set_steering(steer, forzar=True)
        speed = SPEED_ESCAPE
    else:
        set_steering(0, forzar=True)
        speed = SPEED_EXIT

    ritmo = FixedRateLoop(CONTROL_PERIOD_S)
    inicio = time.monotonic()
    move_forward(speed)
    while time.monotonic() - inicio < EXIT_STRAIGHT_S:
        ritmo.wait()
        s = read_scan()
        if s.healthy and s.front is not None and s.front <= FRONT_CRITICAL_MM:
            brake(); set_steering(0, forzar=True)
            raise RecoverableError(f"Frente critico en salida: {s.front:.0f} mm")
        tele.write_row(
            estado=State.EXIT_TURN.name, corner=corner,
            ref=outer_wall[:3],
            front="" if s.front is None else round(s.front),
            vel=speed, lidar_ok=int(s.healthy),
        )
    brake()


                                                                             
# Validate control rules without hardware
def autotest() -> int:
    """Code-specific behavior for autotest."""
    fallos = 0
    esperado = {
        ("clockwise", "GREEN"): (LEFT, "outer"),
        ("clockwise", "RED"): (RIGHT, "inner"),
        ("anticlockwise", "GREEN"): (LEFT, "inner"),
        ("anticlockwise", "RED"): (RIGHT, "outer"),
    }
    print("TABLA DEL REGLAMENTO")
    print("-" * 60)
    for direction, color, ref, etiqueta in rules_table():
        esp = esperado[(direction, color)]
        ok = (ref, etiqueta) == esp
        fallos += 0 if ok else 1
        print(
            f"{'OK ' if ok else 'MAL'}  {direction:<12s} {color:<6s} -> "
            f"pared {ref:<10s} ({etiqueta})"
        )

    print()
    print("SIMETRIA: el volante de cruce solo depende del color")
    print("-" * 60)
    for color in ("GREEN", "RED"):
        h = steer_toward_wall(reference_wall(color, LEFT))
        a = steer_toward_wall(reference_wall(color, RIGHT))
        ok = h == a
        fallos += 0 if ok else 1
        print(
            f"{'OK ' if ok else 'MAL'}  {color:<6s} horario={h:+.0f} "
            f"antihorario={a:+.0f}"
        )

    print()
    print("CONSTANTES OBSOLETAS ELIMINADAS")
    print("-" * 60)
    for nombre in ("PILAR_OBJETIVO_LEJOS_MM", "PILAR_OBJETIVO_CERCA_MM"):
        ok = nombre not in globals()
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {nombre} ausente")

    print()
    print("MARGENES DEL HUECO (pilar 370 mm, robot 120 mm)")
    print("-" * 60)
    for nombre, valor, lo, hi in (
        ("PILAR_OBJETIVO_MM", PILAR_TARGET_MM, 60.0, 295.0),
        ("PILAR_HUECO_MINIMO_MM", PILAR_GAP_MINIMUM_MM, 60.0, 295.0),
        ("PILAR_DISTANCIA_CRITICA_MM", PILAR_DISTANCE_CRITICA_MM, 60.0, 295.0),
    ):
        ok = lo <= valor <= hi
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {nombre} = {valor:.0f} en [{lo:.0f},{hi:.0f}]")
    ok = PILAR_SIGNAL_MAX_MM > PILAR_TARGET_MM
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  PILAR_SENAL_MAX_MM "
        f"({PILAR_SIGNAL_MAX_MM:.0f}) > objetivo"
    )

    print()
    print("CORTE FRONTAL DURANTE LA EVASION")
    print("-" * 60)
    casos = [
        ("sin pilar", False, None, None, 0.0, FRONT_CRITICAL_MM),
        ("pilar centrado y cerca", True, "RED", 0.57, 0.62, PILAR_DIST_STOP_MM),
        ("pilar ya a un lado", True, "RED", 0.15, 0.70, FRONT_CRITICAL_MM),
        ("pilar centrado pero lejos", True, "RED", 0.50, 0.35, FRONT_CRITICAL_MM),
        ("color distinto al activo", True, "GREEN", 0.50, 0.70, FRONT_CRITICAL_MM),
        ("sin x (vision caida)", True, "RED", None, 0.50, FRONT_CRITICAL_MM),
    ]
    for desc, activo, color_v, x, bottom, esp in casos:
        v = Vision(color_v, x, bottom, 0.0, True, 0.0, 0)
        got = front_limit(activo, v, "RED")
        ok = got == esp
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {desc:<28s} -> {got:.0f} mm")

    print()
    print("FILTRO DE SALTOS IMPOSIBLES")
    print("-" * 60)
    saltos = [
        ("salto real del 29 ago (455->665 en 0.05s)", 665.0, 455.0, 0.05, True),
        ("movimiento normal (455->460 en 0.05s)", 460.0, 455.0, 0.05, False),
        ("cruce agresivo (500->460 en 0.20s)", 460.0, 500.0, 0.20, False),
        ("primera lectura del tramo", 500.0, None, 0.05, False),
        ("sin lectura", None, 500.0, 0.05, False),
    ]
    for desc, d, prev, dt, esp in saltos:
        got = impossible_jump(d, prev, dt)
        ok = got == esp
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {desc:<42s} -> {'rechaza' if got else 'acepta'}")

    print()
    print("V5.1: EL RECHAZO SE SOSTIENE DURANTE TODO EL BARRIDO")
    print("-" * 60)
    _f = inspect.getsource(drive_section)
    pruebas_v51 = [
        (
            "la decision se toma una sola vez, al llegar el barrido",
            "scan_rechazado = salto_imposible(d, d_valida, now - t_valida)" in _f,
        ),
        (
            "ya NO se evalua el salto pegado al gate de seq (bug V5.0)",
            "if s.seq != seq_prev and salto_imposible(" not in _f,
        ),
        (
            "la sustitucion by d_valida no depende del gate de seq",
            "if scan_rechazado:\n            d = d_valida" in _f,
        ),
        (
            "el flag se inicializa antes del lazo",
            "scan_rechazado = False\n" in _f,
        ),
        (
            "d_valida solo se latchea cuando el barrido NO fue rechazado",
            "elif d is not None:\n            d_valida = d" in _f,
        ),
        (
            "la pared critica se evalua despues del filtro",
            _f.find("if scan_rechazado:") < _f.find("critica_confirm += 1"),
        ),
    ]
    for desc, ok in pruebas_v51:
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {desc}")

                                                                            
                                                                        
                                                                             
                                                                 
    d_valida_t, t_valida_t = 441.0, -CONTROL_PERIOD_S
    saltos_t, rech_t, critica_t = 0, False, 0
    disparo = False
    for i in range(7):
        now = i * CONTROL_PERIOD_S
        seq_nuevo = (i == 0)
        d_t = 136.0
        if seq_nuevo:
            rech_t = impossible_jump(d_t, d_valida_t, now - t_valida_t)
            if rech_t:
                saltos_t += 1
                if saltos_t > JUMP_CICLOS_MAX:
                    saltos_t, rech_t = 0, False
            else:
                saltos_t = 0
        if rech_t:
            d_t = d_valida_t
        else:
            d_valida_t, t_valida_t = d_t, now
        critica_t = critica_t + 1 if d_t <= DISTANCE_CRITICA_MM else 0
        if critica_t >= DISTANCE_CRITICA_CONFIRMATIONS:
            disparo = True
    fallos += 0 if not disparo else 1
    print(
        f"{'OK ' if not disparo else 'MAL'}  barrido malo 441->136 leido 7 veces "
        f"-> {'NO dispara' if not disparo else 'DISPARA'} pared critica"
    )

    print()
    print("REVERSA DE RECUPERACION: giro segun la pared")
    print("-" * 60)
    for wall, esperado_signo in ((RIGHT, -1.0), (LEFT, +1.0)):
        got = steer_toward_wall(wall) * RECOVER_ANGLE_DEG
        ok = math.copysign(1.0, got) == esperado_signo
        fallos += 0 if ok else 1
        print(
            f"{'OK ' if ok else 'MAL'}  pared {wall:<10s} -> volante "
            f"{got:+.0f} deg (morro sale al lado contrario)"
        )

    print()
    print("V4.6: LA FASE CIEGA SOLO CORRE CON EL PILAR DELANTE")
    print("-" * 60)
    fuente = inspect.getsource(drive_section)

    pruebas_fuente = [
        (
            "la salida es: carril activo y pilar ya no delante",
            "salida_activa = carril_activo and not pilar_activo" in fuente,
        ),
        (
            "el carril se sostiene durante el refractario",
            "pilar_activo or now < pilar_bloqueado_hasta" in fuente,
        ),
        (
            "el permiso de fase ciega es pilar_activo",
            "pilar_activo,      # la fase ciega solo con el pilar delante" in fuente,
        ),
        (
            "ya NO se endereza al rumbo en la salida (bug de la V4.5)",
            "KA_RUMBO * (rumbo_ref - mpu.rumbo())" not in fuente,
        ),
        (
            "refractario asignado una sola vez",
            fuente.count("pilar_bloqueado_hasta = now + PILAR_REFRACTARIO_S") == 1,
        ),
        (
            "refractario abierto antes de decidir la referencia",
            0 <= fuente.find("pilar_bloqueado_hasta = now + PILAR_REFRACTARIO_S")
            < fuente.find("carril_deseado ="),
        ),
        (
            "sin fases de recuperacion post-pilar",
            "CONTRAGIRO" not in fuente,
        ),
        (
            "el refractario sigue bloqueando la esquina falsa",
            "pilar_cerca = pilar_activo or now < pilar_bloqueado_hasta" in fuente,
        ),
    ]
    for desc, ok in pruebas_fuente:
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {desc}")

    print()
    print("COMPORTAMIENTO DEL LAZO EN CARRIL Y EN SALIDA")
    print("-" * 60)
                                                                           
    mpu._ready = True
    signo_previo = mpu.signo
    mpu.signo = 1.0
                                                                          
                                                                             
                                                                            
                                                                             
    ciega_esperada = min(
        PILAR_KA_HEADING * PILAR_HEADING_OFFSET_MAX_DEG, PILAR_STEERING_CASCADA_MAX_DEG
    )
    casos = [
                                                   
        ("pilar delante, sin lectura -> se compromete",
         None, True, ciega_esperada),
        ("pilar delante, lectura falsa 780 -> se compromete",
         780.0, True, ciega_esperada),
        ("en salida, sin lectura -> volante quieto",
         None, False, 0.0),
        ("en salida, lectura falsa 780 -> volante quieto",
         780.0, False, 0.0),
    ]
    for desc, d_caso, permitir, esp in casos:
        cmd, _ = straight_control(
            d_caso, None, RIGHT, None, 0.0, 0.0, rumbo_ref=0.0,
            objetivo=PILAR_TARGET_MM, giro_max=PILAR_STEERING_CASCADA_MAX_DEG,
            carril=True, permitir_ciega=permitir,
        )
        ok = abs(abs(cmd) - esp) < 0.51
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {desc:<48s} cmd={cmd:+6.1f}")

                                                                           
    cmd, _ = straight_control(
        350.0, None, RIGHT, None, 0.0, 0.0, rumbo_ref=0.0,
        objetivo=PILAR_TARGET_MM, giro_max=PILAR_STEERING_CASCADA_MAX_DEG,
        carril=True, permitir_ciega=False,
    )
    ok = cmd < -1.0
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  en salida con d=350 -> sigue cruzando"
        f"{'':21s} cmd={cmd:+6.1f}"
    )
    mpu._ready = False
    mpu.signo = signo_previo

    print()
    print("CRITERIO DE REBASADO: lo que significa x=0.12 en el piso")
    print("-" * 60)
    for dist_mm in (300.0, 400.0, 600.0):
        ang = math.atan((PILAR_X_PASSED_RED - 0.5) * CAM_W / CAMARA_FOCAL_PX)
        lat = dist_mm * math.tan(abs(ang))
        holgura = lat - 50.0 - 60.0
        print(
            f"     pilar a {dist_mm:.0f} mm -> holgura lateral "
            f"{holgura:+.0f} mm (sigue adelante)"
        )
    print("OK   by eso el cruce se termina durante el refractario")

    print()
    print("V4.7: RECTA NORMAL SIN LECTURA DE PARED")
    print("-" * 60)
    mpu._ready = True
    signo_previo2 = mpu.signo
    mpu.signo = 1.0

                                                                         
                                                                           
    mpu._heading = -21.0
    cmd, _ = straight_control(
        None, None, RIGHT, None, 0.0, 0.0, rumbo_ref=0.0, carril=False
    )
    ok = cmd > 1.0
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  torcido -21 deg y sin pared -> se endereza"
        f"      cmd={cmd:+6.1f}"
    )

                                                     
    mpu._heading = 0.0
    cmd, _ = straight_control(
        None, None, RIGHT, None, 0.0, 0.0, rumbo_ref=0.0, carril=False
    )
    ok = abs(cmd) < 0.51
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  alineado y sin pared -> sigue derecho"
        f"          cmd={cmd:+6.1f}"
    )

                                                                       
                                                                           
    mpu._heading = -21.0
    cmd, _ = straight_control(
        None, None, RIGHT, None, 0.0, 0.0, rumbo_ref=0.0,
        objetivo=PILAR_TARGET_MM, giro_max=PILAR_STEERING_CASCADA_MAX_DEG,
        carril=True, permitir_ciega=False,
    )
    ok = abs(cmd) < 0.51
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  en salida y sin pared -> NO endereza"
        f"           cmd={cmd:+6.1f}"
    )

                                                      
    mpu._heading = -80.0
    cmd, _ = straight_control(
        None, None, RIGHT, None, 0.0, 0.0, rumbo_ref=0.0, carril=False
    )
    ok = abs(cmd - STEERING_CONTROL_MAX_DEG) < 0.51
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  enderezamiento limitado a "
        f"{STEERING_CONTROL_MAX_DEG:.0f} deg{'':10s} cmd={cmd:+6.1f}"
    )

    mpu._heading = 0.0
    mpu._ready = False
    mpu.signo = signo_previo2

                                                      
    cmd, _ = straight_control(
        None, None, RIGHT, None, 0.0, 0.0, rumbo_ref=0.0, carril=False
    )
    ok = abs(cmd) < 0.51
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  sin MPU y sin pared -> volante cero"
        f"           cmd={cmd:+6.1f}"
    )

    print()
    print("V4.8: EL CRUCE NO PUEDE PASAR DEL ANGULO DONDE EL LIDAR VE")
    print("-" * 60)
    ok = PILAR_HEADING_OFFSET_MAX_DEG <= LIDAR_ANGLE_MAX_USABLE_DEG
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  tope de cruce {PILAR_HEADING_OFFSET_MAX_DEG:.0f} deg "
        f"<= angulo util maximo {LIDAR_ANGLE_MAX_USABLE_DEG:.0f} deg"
    )

                                                                              
                                                                            
    V_REAL_MM_S = 238.0                                             
    T_CRUCE_S = 2.80                                             
    D_INICIAL_MM = 463.0                                               
    CENTRO_MAX_MM = 310.0                                                
    lateral = V_REAL_MM_S * math.sin(math.radians(PILAR_HEADING_OFFSET_MAX_DEG))
    final = D_INICIAL_MM - lateral * T_CRUCE_S
    ok = final < CENTRO_MAX_MM
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  a {lateral:.0f} mm/s durante {T_CRUCE_S:.1f} s "
        f"termina en {final:.0f} mm (debe ser < {CENTRO_MAX_MM:.0f})"
    )
    ok = final > PILAR_DISTANCE_CRITICA_MM
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  y termina por encima del limite critico "
        f"({PILAR_DISTANCE_CRITICA_MM:.0f} mm)"
    )

    print()
    print("V4.9: EL CRUCE ALCANZA EL ANGULO QUE SE LE PIDE")
    print("-" * 60)
    ok = PILAR_KA_HEADING > KA_HEADING
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  ganancia del pilar {PILAR_KA_HEADING:.2f} > "
        f"la de la recta {KA_HEADING:.2f}"
    )

                                                                         
                                                                              
                                                                         
    K_VEH_DEG_POR_MM = 0.524 / 238.0
                                                                          
                                                                       
                                                                          
                                                                            
                                                                          
                                                                            
                                                                            
                                                      
                                                                           
                                                                          
                                                                           
                                                                           
    X_DISPONIBLE_MM = 900.0
    CRUCE_MINIMO_MM = 310.0

    rel, lat, dx = 0.0, 0.0, 0.5
    for _ in range(int(X_DISPONIBLE_MM / dx)):
        delta = clamp(
            PILAR_KA_HEADING * (PILAR_HEADING_OFFSET_MAX_DEG - rel),
            -PILAR_STEERING_CASCADA_MAX_DEG, PILAR_STEERING_CASCADA_MAX_DEG,
        )
        rel += K_VEH_DEG_POR_MM * delta * dx
        lat += math.tan(math.radians(rel)) * dx
    ok = lat >= CRUCE_MINIMO_MM
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  en {X_DISPONIBLE_MM:.0f} mm cruza {lat:.0f} mm "
        f"(minimo {CRUCE_MINIMO_MM:.0f})"
    )
    ok = rel <= LIDAR_ANGLE_MAX_USABLE_DEG + 0.51
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  y sin pasarse del angulo ciego: llega a "
        f"{rel:.1f} deg"
    )

    print()
    print("LIMITES DUROS: todos reaccionan a ESTAR DEMASIADO CERCA")
    print("-" * 60)
                                                                            
                                                                          
                                                                   
    cmd_cerca, _ = straight_control(
        250.0, None, RIGHT, None, 0.0, 0.0, rumbo_ref=None, carril=False
    )
    ok = cmd_cerca > 0.0
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  d=250 (bajo el escape de "
        f"{DISTANCE_ESCAPE_MM:.0f}) -> se separa   cmd={cmd_cerca:+6.1f}"
    )

    print()
    print("V5.0: FILTRO DE PLAUSIBILIDAD GEOMETRICA DEL PILAR")
    print("-" * 60)
                                                                           
                                                                          
                                                                       
    casos_plausible = [
        ("verde  6 sep  400 mm    ", 0.72, 3400.0, True),
        ("verde  6 sep  900 mm    ", 0.42, 757.0, True),
        ("verde  6 sep 1200 mm    ", 0.35, 404.0, True),
        ("rojo   6 sep  400 mm    ", 0.72, 3905.0, True),
        ("verde muy cerca (ROI)   ", 0.85, 9000.0, True),
        ("FANTASMA del 30 ago     ", 0.49, 302.0, False),
        ("blob pegado al horizonte", 0.17, 900.0, False),
    ]
    for nombre, bottom, area, esperado in casos_plausible:
        real = pillar_is_plausible(bottom, area)
        ok = real == esperado
        fallos += 0 if ok else 1
        d_a = PILAR_K_AREA_MM / math.sqrt(area)
        d_b = distance_from_bottom(bottom)
        veredicto = "acepta" if real else "RECHAZA"
        razon_txt = "---- " if d_b is None else f"{d_a / d_b:4.2f}"
        print(
            f"{'OK ' if ok else 'MAL'}  {nombre} d_b="
            + ("----" if d_b is None else f"{d_b:4.0f}")
            + f" d_a={d_a:4.0f} razon={razon_txt} -> {veredicto}"
        )

    ok = PILAR_RAZON_MIN < 1.0 < PILAR_RAZON_MAX
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  la ventana [{PILAR_RAZON_MIN:.2f}, "
        f"{PILAR_RAZON_MAX:.2f}] contiene el acuerdo perfecto (1.00)"
    )

    fuente_tramo = inspect.getsource(drive_section)
    ok = "pilar_plausible(v.bottom, v.area)" in fuente_tramo
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  el filtro se aplica en la activacion "
        f"de recorrer_tramo"
    )

    print()
    print("V5.0: LA REVERSA SE GIRA POR EL OBSTACULO, NO POR LA REFERENCIA")
    print("-" * 60)

    def _scan_lados(izq, der):
        return Scan(
            healthy=True, front=180.0, left=izq, right=der,
            yaw_left=None, yaw_right=None, seq=0, age=0.0,
        )

    casos_reversa = [
        (
            "evadiendo pilar, ref DERECHA -> morro sale a la derecha",
            _scan_lados(600.0, 200.0), RIGHT, True, LEFT,
        ),
        (
            "evadiendo pilar, ref IZQUIERDA -> morro sale a la izquierda",
            _scan_lados(200.0, 600.0), LEFT, True, RIGHT,
        ),
        (
            "sin pilar, izquierda cerrada -> morro sale a la derecha",
            _scan_lados(150.0, 700.0), RIGHT, False, LEFT,
        ),
        (
            "sin pilar, derecha cerrada -> morro sale a la izquierda",
            _scan_lados(700.0, 150.0), RIGHT, False, RIGHT,
        ),
        (
            "sin pilar y sin lecturas -> reversa recta (None)",
            _scan_lados(None, None), RIGHT, False, None,
        ),
    ]
    for nombre, scan, ref, carril, esperado in casos_reversa:
        real = wall_for_front_recovery(scan, ref, carril)
        ok = real == esperado
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {nombre}")

                                                                             
                                                                             
                                                                  
    ref_bug = RIGHT
    viejo = ref_bug
    nuevo = wall_for_front_recovery(_scan_lados(600.0, 200.0), ref_bug, True)
    ok = nuevo != viejo
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  el caso del 30 ago cambia de "
        f"{viejo} a {nuevo}"
    )

    fuente_tramo = inspect.getsource(drive_section)
    ok = (
        "pared_para_reversa_frontal(s, pared_ref, carril_activo)"
        in fuente_tramo
    )
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  el frente critico ya no pasa pared_ref "
        f"directo"
    )

    print()
    print("V5.6 (1): FORMA DE PILAR - LLENADO DE LA CAJA")
    print("-" * 60)
                                                                         
                                                                 
    casos_forma = [
        ("pilar lejano 1200 mm  16x33 px", 16, 33, 455.0, True),
        ("pilar medio   600 mm  33x65 px", 33, 65, 1870.0, True),
        ("pilar cerca   400 mm  49x98 px", 49, 98, 3960.0, True),
        ("pilar cortado by el ROI      ", 49, 60, 2530.0, True),
        ("linea naranja horizontal      ", 180, 22, 3400.0, False),
        ("linea naranja en diagonal     ", 90, 110, 3400.0, False),
        ("mancha delgada en la esquina  ", 40, 70, 1000.0, False),
    ]
    for nombre, w, h, area, esperado in casos_forma:
        real = pillar_shape(float(w), float(h), area)
        ok = real == esperado
        fallos += 0 if ok else 1
        print(
            f"{'OK ' if ok else 'MAL'}  {nombre}  llenado="
            f"{area / (w * h):.2f} -> {'pilar' if real else 'descartado'}"
        )

                                                                            
                                                                        
    def _cand(cx, bottom, area):
        return (area * (0.5 + bottom), cx, bottom, area)

    impostor = _cand(0.05, 0.72, 302.0)                                  
    verde_real = _cand(0.32, 0.35, 404.0)                                            
    color, x, bottom, area = select_pillar([impostor], [verde_real])
    ok = color == "GREEN" and abs(bottom - 0.35) < 1e-6
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  impostor rojo + verde real -> se publica "
        f"{color} b={bottom:.2f} (V5.1 publicaba RED)"
    )

    color, _x, _b, _a = select_pillar([impostor], [])
    ok = color == "RED"
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  impostor solo -> se publica igual para que "
        f"salga el aviso DESCARTADO"
    )

    color, _x, _b, _a = select_pillar([], [])
    ok = color is None
    fallos += 0 if ok else 1
    print(f"{'OK ' if ok else 'MAL'}  sin candidatos -> None")

    rojo_cerca = _cand(0.70, 0.72, 3905.0)                                  
    color, _x, _b, _a = select_pillar([rojo_cerca], [verde_real])
    ok = color == "RED"
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  dos pilares validos -> se evade primero el "
        f"mas cercano ({color})"
    )

    print()
    print("V6.3: CENTRADO EN EL CORREDOR SIN SABER EL SENTIDO")
    print("-" * 60)

    def _lados(izq, der):
        return Scan(
            front=None, left=izq, right=der, healthy=True,
            yaw_left=None, yaw_right=None, seq=0, age=0.0,
        )

    casos_centro = [
        ("pegado a la izquierda (276/749) -> derecha", 276.0, 749.0, -1),
        ("pegado a la derecha  (749/276) -> izquierda", 749.0, 276.0, 1),
        ("centrado (500/500) -> recto              ", 500.0, 500.0, 0),
        ("esquina: inner abierto (2316/450)     ", 2316.0, 450.0, 0),
        ("una pared no return                    ", None, 450.0, 0),
    ]
    for nombre, izq, der, signo in casos_centro:
        cmd = corridor_centering(_lados(izq, der))
        real = 0 if abs(cmd) < 0.01 else (1 if cmd > 0 else -1)
        ok = real == signo and abs(cmd) <= CENTRADO_MAX_DEG + 0.01
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {nombre}  cmd={cmd:+5.1f}")

    fuente_c = inspect.getsource(determine_direction)
    ok = "cmd = centrado_corredor(s)" in fuente_c
    fallos += 0 if ok else 1
    print(f"{'OK ' if ok else 'MAL'}  BUSCAR_SENTIDO se centra en vez de ir recto")

    print()
    print("V6.2: UN PILAR NO PUEDE HACERSE PASAR POR ESQUINA")
    print("-" * 60)
    fuente_sent = inspect.getsource(determine_direction)
    checks_sent = [
        (
            "el refractario del pilar existe en detectar_sentido",
            "pilar_bloqueado_hasta = now + PILAR_REFRACTARIO_S" in fuente_sent,
        ),
        (
            "la esquina exige haber visto el frente despejado",
            "and frente_despejado_visto" in fuente_sent,
        ),
        (
            "un pilar delante borra el frente despejado",
            "frente_despejado_visto = False" in fuente_sent,
        ),
        (
            "la esquina se bloquea con el pilar cerca",
            "pilar_cerca = pilar_activo or now < pilar_bloqueado_hasta"
            in fuente_sent,
        ),
        (
            "el contador se reinicia cuando no se puede contar",
            fuente_sent.count("confirm = 0") >= 2,
        ),
    ]
    for nombre, cond in checks_sent:
        fallos += 0 if cond else 1
        print(f"{'OK ' if cond else 'MAL'}  {nombre}")

    print()
    print("V6.1: LA FASE CIEGA DEL CRUCE TIENE TECHO")
    print("-" * 60)
    fuente_ciego = inspect.getsource(drive_section)
    checks_ciego = [
        (
            "la ceguera ya NO exime by carril activo",
            "if carril_activo or d is not None:" not in fuente_ciego,
        ),
        (
            "el reloj corre siempre que no hay pared",
            "if d is not None:\n            pared_perdida_desde = None"
            in fuente_ciego,
        ),
        (
            "el plazo del carril es mas largo, no infinito",
            "CARRIL_CIEGO_TIMEOUT_S if carril_activo" in fuente_ciego,
        ),
        (
            "el plazo del carril supera la fase ciega real (1.2 s)",
            CARRIL_BLIND_TIMEOUT_S >= 1.5,
        ),
        (
            "pero no es indefinido",
            CARRIL_BLIND_TIMEOUT_S <= 3.0,
        ),
    ]
    for nombre, cond in checks_ciego:
        fallos += 0 if cond else 1
        print(f"{'OK ' if cond else 'MAL'}  {nombre}")

    print()
    print("V5.9: EL REBASADO DESCUENTA EL GIRO DEL ROBOT")
    print("-" * 60)
                                                                            
                                                                
    casos_reb = [
        ("verde x=0.88 con rel=+18.9 -> NO rebasado", "GREEN", 0.88, 18.9, False),
        ("verde x=0.88 sin giro      -> rebasado   ", "GREEN", 0.88, 0.0, True),
        ("verde x=0.30 al detectarlo -> NO rebasado", "GREEN", 0.30, 0.0, False),
        ("rojo  x=0.12 con rel=-18.9 -> NO rebasado", "RED", 0.12, -18.9, False),
        ("rojo  x=0.12 sin giro      -> rebasado   ", "RED", 0.12, 0.0, True),
        ("sin giroscopio: como antes ", "GREEN", 0.88, None, True),
    ]
    for nombre, color, x, rel, esperado in casos_reb:
        real = pillar_passed(color, x, rel)
        ok = real == esperado
        fallos += 0 if ok else 1
        xc = rotation_compensated_x(x, rel)
        print(f"{'OK ' if ok else 'MAL'}  {nombre}  x_corr={xc:.2f}")

    fuente_reb = inspect.getsource(drive_section)
    ok = "pilar_rebasado(pilar_color, x_actual, rel_pilar)" in fuente_reb
    fallos += 0 if ok else 1
    print(f"{'OK ' if ok else 'MAL'}  recorrer_tramo pasa el giro al rebasado")

    print()
    print("V5.6 (2): SENTIDO - UN LADO SIN RETORNO ES EL ABIERTO")
    print("-" * 60)
                                                    
    casos_sentido = [
        ("inner izq se abrio a 2800", 2800.0, 286.0, 2800.0, 260.0, RIGHT),
        ("inner der se abrio a 2800", 286.0, 2800.0, 260.0, 2800.0, LEFT),
        ("izq no return, der 300 mm ", 480.0, 300.0, None, 300.0, RIGHT),
        ("der no return, izq 300 mm ", 300.0, 480.0, 300.0, None, LEFT),
        ("los dos devuelven, izq lejos", 700.0, 300.0, 700.0, 300.0, RIGHT),
        ("ninguno devuelve            ", 900.0, 300.0, None, None, RIGHT),
    ]
    for nombre, mi, md, i, d, esperado in casos_sentido:
        real = determine_outer_wall(mi, md, i, d)
        ok = real == esperado
        fallos += 0 if ok else 1
        print(f"{'OK ' if ok else 'MAL'}  {nombre} -> exterior {real}")

                                                                          
                                                                   
    viejo_izq = -1.0                                                      
    viejo = RIGHT if viejo_izq > 300.0 else LEFT
    nuevo = determine_outer_wall(480.0, 300.0, None, 300.0)
    ok = nuevo != viejo
    fallos += 0 if ok else 1
    print(
        f"{'OK ' if ok else 'MAL'}  con el lado abierto mudo cambia de "
        f"{viejo} a {nuevo}"
    )

    fuente_sentido = inspect.getsource(determine_direction)
    ok = "decidir_pared_outer(max_izq, max_der" in fuente_sentido
    fallos += 0 if ok else 1
    print(f"{'OK ' if ok else 'MAL'}  detectar_sentido usa la regla nueva")

    fuente_camara = inspect.getsource(_camera_session)
    ok = "elegir_pilar(" in fuente_camara and "blob_color(" not in fuente_camara
    fallos += 0 if ok else 1
    print(f"{'OK ' if ok else 'MAL'}  la camara publica por elegir_pilar")

    print()
    print(f"VERSION VERIFICADA: {VERSION}")
    print("TODO OK" if fallos == 0 else f"{fallos} FALLOS")
    return 0 if fallos == 0 else 1


                                                                             
def verify_heading(corners: int) -> None:
    """Code-specific behavior for verificar rumbo."""
    if not mpu.ready or mpu.signo is None:
        return
    esperado = 90.0 * corners
    real = abs(mpu.heading_total())
    error = real - esperado
    aviso = "" if abs(error) <= VERIFICACION_HEADING_TOLERANCE_DEG else "  <-- REVISAR"
    print(
        f"VERIFICACION RUMBO: esperado {esperado:.0f} deg, "
        f"medido {real:.0f} deg, error {error:+.0f} deg{aviso}"
    )


# Main competition sequence
def main() -> None:
    hilo_lidar = threading.Thread(target=lidar_worker, daemon=True)
    hilo_camara = threading.Thread(target=camera_worker, daemon=True)
    corners = 0
    try:
        stop_motion(); set_steering(0, forzar=True)
        print(
            f"WRO 2026 | FSM PILARES {VERSION} | {LAPS} vueltas "
            f"= {NUM_CORNERS} esquinas"
        )
        print("REGLA: la evasion sigue la pared del lado hacia el que se cruza")
        print("V5.1: rechazo de salto sostenido by barrido")
        print("V5.6: vision multi-blob + sentido by lado abierto")
        print("V5.7: modelo de bottom con horizonte, camara reapuntada 6 sep")
        tele.open()

                                                              
        mpu.start()
        lidar_stop.clear(); vision_stop.clear()
        hilo_lidar.start(); hilo_camara.start()
        wait_for_lidar(); wait_for_camera()
        print("LiDAR y camara listos")

        outer_wall = run_state("BUSCAR_SENTIDO", determine_direction)

        while corners < NUM_CORNERS:
            if corners > 0:
                run_state("RECTO", drive_section, outer_wall, corners + 1)
            run_state("ESQUINA", corner_maneuver, outer_wall, corners + 1)
            corners += 1
            print(f"ESQUINA {corners}/{NUM_CORNERS}")
            verify_heading(corners)

        if SECTION_FINAL_S > 0:
            run_state(
                "TRAMO_FINAL", drive_section,
                outer_wall, corners, SECTION_FINAL_S,
            )

        brake(); set_steering(0, forzar=True)
        print(f"FSM TERMINADO: {NUM_CORNERS} esquinas completadas")

    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except Fatal as exc:
        print("FINAL STOP:", exc)
    except Exception as exc:
        print("STOP:", type(exc).__name__, exc)
    finally:
        stop_motion(); set_steering(0, forzar=True)
        print(
            f"RESUMEN: {corners}/{NUM_CORNERS} esquinas, "
            f"{recoveries} incidentes recuperados"
        )
        mpu.stop()
        lidar_stop.set(); vision_stop.set()
        with lidar_condition:
            lidar_condition.notify_all()
        if hilo_lidar.is_alive():
            hilo_lidar.join(timeout=2.0)
        if hilo_camara.is_alive():
            hilo_camara.join(timeout=2.0)
        tele.close()
        try:
            motor.close(); servo.close()
        except Exception:
            pass
        print("System stopped")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WRO 2026 Obstaculos")
    parser.add_argument("--sim", action="store_true", help="simulator without hardware")
    parser.add_argument("--autotest", action="store_true", help="verify control logic")
    parser.add_argument("--laps", type=int, default=None)
    parser.add_argument("--sin-telemetria", action="store_true")
    args = parser.parse_args()
    if args.autotest:
        sys.exit(autotest())
    if args.vueltas is not None:
        LAPS = args.vueltas
        NUM_CORNERS = LAPS * CORNERS_POR_LAP
    if args.sin_telemetria:
        TELEMETRIA_ENABLED = False
    main()




