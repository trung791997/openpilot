import math
import multiprocessing
import os
import numpy as np

from abc import ABC, abstractmethod
from collections import namedtuple

# SIM_CAMERA=mici renders the comma 4's os04c10 cameras (1344x760; road f=1141.5, wide pinhole f=425.25) and publishes
# sensor os04c10, so modeld/calibrationd use the comma 4's intrinsics; default (tici) keeps upstream's AR0231 geometry.
# Both are pinhole renders: the real wide lens is a fisheye, which MetaDrive does not model.
SIM_CAMERA = os.environ.get("SIM_CAMERA", "tici")
if SIM_CAMERA == "mici":
  W, H = 1344, 760
  ROAD_FOCAL, WIDE_FOCAL, CAMERA_SENSOR = 1522.0 * 3 / 4, 567.0 / 4 * 3, "os04c10"
else:
  W, H = 1928, 1208
  ROAD_FOCAL, WIDE_FOCAL, CAMERA_SENSOR = 2648.0, 567.0, None
ROAD_HFOV = 2 * math.degrees(math.atan(W / 2 / ROAD_FOCAL))
WIDE_HFOV = 2 * math.degrees(math.atan(W / 2 / WIDE_FOCAL))
if SIM_CAMERA != "mici":
  ROAD_HFOV, WIDE_HFOV = 40.0, 120.0  # upstream's lens values (f=567 gives 119.1); kept so earlier episodes stay comparable


vec3 = namedtuple("vec3", ["x", "y", "z"])

class GPSState:
  def __init__(self):
    self.latitude = 0
    self.longitude = 0
    self.altitude = 0

  def from_xy(self, xy):
    """Simulates a lat/lon from an xy coordinate on a plane, for simple simulation. TODO: proper global projection?"""
    BASE_LAT = 32.75308505188913
    BASE_LON = -117.2095393365393
    DEG_TO_METERS = 100000

    self.latitude = float(BASE_LAT + xy[0] / DEG_TO_METERS)
    self.longitude = float(BASE_LON + xy[1] / DEG_TO_METERS)
    self.altitude = 0


class IMUState:
  def __init__(self):
    self.accelerometer: vec3 = vec3(0,0,0)
    self.gyroscope: vec3 = vec3(0,0,0)
    self.bearing: float = 0


class SimulatorState:
  def __init__(self):
    self.valid = False
    self.is_engaged = False
    self.ignition = True

    self.velocity: vec3 = None
    self.bearing: float = 0
    self.gps = GPSState()
    self.imu = IMUState()

    self.steering_angle: float = 0

    self.user_gas: float = 0
    self.user_brake: float = 0
    self.user_torque: float = 0

    self.cruise_button = 0

    self.left_blinker = False
    self.right_blinker = False

  @property
  def speed(self):
    return math.sqrt(self.velocity.x ** 2 + self.velocity.y ** 2 + self.velocity.z ** 2)


class World(ABC):
  def __init__(self, dual_camera):
    self.dual_camera = dual_camera

    self.image_lock = multiprocessing.Semaphore(value=0)
    self.road_image = np.zeros((H, W, 3), dtype=np.uint8)
    self.wide_road_image = np.zeros((H, W, 3), dtype=np.uint8)

    self.exit_event = multiprocessing.Event()
    self.blinker = 0  # turn signal the world asks for (+1 left, -1 right; metadrive SIM_BLINKER=auto)
    self.plant_cmd = None  # SIM_PLANT=civic: (wheel angle deg, accel request, vehicle model params)

  @abstractmethod
  def apply_controls(self, steer_sim, throttle_out, brake_out):
    pass

  @abstractmethod
  def tick(self):
    pass

  @abstractmethod
  def read_state(self):
    pass

  @abstractmethod
  def read_sensors(self, simulator_state: SimulatorState):
    pass

  @abstractmethod
  def read_cameras(self):
    pass

  @abstractmethod
  def close(self, reason: str):
    pass

  @abstractmethod
  def reset(self):
    pass
