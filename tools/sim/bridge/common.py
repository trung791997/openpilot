import math
import os
import signal
import threading
import functools
import traceback
import numpy as np

from collections import namedtuple
from enum import Enum
from multiprocessing import Process, Queue, Value
from abc import ABC, abstractmethod

from opendbc.car.honda.values import CruiseButtons
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors
from openpilot.tools.sim.lib.steer_model import SteerModel

CIVIC_PLANT = os.getenv("SIM_PLANT", "metadrive") == "civic"
METADRIVE_CURV_PER_DEG = 0.00055  # measured road curvature per degree sent to MetaDrive (steer_ratio 8 in metadrive_process)

QueueMessage = namedtuple("QueueMessage", ["type", "info"], defaults=[None])

class QueueMessageType(Enum):
  START_STATUS = 0
  CONTROL_COMMAND = 1
  TERMINATION_INFO = 2
  CLOSE_STATUS = 3

def control_cmd_gen(cmd: str):
  return QueueMessage(QueueMessageType.CONTROL_COMMAND, cmd)

def rk_loop(function, hz, exit_event: threading.Event):
  rk = Ratekeeper(hz, None)
  while not exit_event.is_set():
    function()
    rk.keep_time()


class SimulatorBridge(ABC):
  TICKS_PER_FRAME = 5
  CRUISE_KEY_FRAMES = 10

  def __init__(self, dual_camera, high_quality):
    set_params_enabled()
    params = Params()
    params.put_bool("AlphaLongitudinalEnabled", True)

    self.rk = Ratekeeper(100, None)

    self.dual_camera = dual_camera
    self.high_quality = high_quality

    self._exit_event: threading.Event | None = None
    self._threads = []
    self._keep_alive = True
    self.started = Value('i', False)
    signal.signal(signal.SIGTERM, self._on_shutdown)
    self.simulator_state = SimulatorState()

    self.world: World | None = None

    self.cruise_key = 0
    self.cruise_key_frames = 0
    self.past_startup_engaged = False
    self.moved_once = False
    self.startup_button_prev = True
    # SIM_CRUISE_KPH: after engaging, tap RES_ACCEL until the set speed reaches this (km/h).
    # Engagement leaves it near 9 km/h, where the driving model crawls and loses the lanes in curves.
    self.target_cruise_kph = float(os.getenv("SIM_CRUISE_KPH", "0"))
    # SIM_STEER_MODEL: steer through the car's own torque command and a steering response fitted from its
    # rlogs (tools/sim/eps_fit.py), so the lateral tune and EPS are in the loop. Needs SIM_CAR_CONFIG.
    steer_model = os.getenv("SIM_STEER_MODEL")
    self.steer_model = SteerModel(steer_model) if steer_model else None
    self.vehicle_model = None  # built from carParams on first use (torque mode)

    self.test_run = False

  def _on_shutdown(self, signal, frame):
    self.shutdown()

  def shutdown(self):
    self._keep_alive = False

  def bridge_keep_alive(self, q: Queue, retries: int):
    try:
      self._run(q)
    except Exception:
      q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, traceback.format_exc()))
      raise
    finally:
      self.close("bridge terminated")

  def close(self, reason):
    self.started.value = False

    if self._exit_event is not None:
      self._exit_event.set()

    if self.world is not None:
      self.world.close(reason)

  def run(self, queue, retries=-1):
    bridge_p = Process(name="bridge", target=self.bridge_keep_alive, args=(queue, retries))
    bridge_p.start()
    return bridge_p

  def print_status(self):
    print(
    f"""
State:
Ignition: {self.simulator_state.ignition} Engaged: {self.simulator_state.is_engaged}
    """)

  @abstractmethod
  def spawn_world(self, q: Queue) -> World:
    pass

  def _run(self, q: Queue):
    self.world = self.spawn_world(q)
    self.world.external_steer = self.steer_model is not None

    self.simulated_car = SimulatedCar()
    self.simulated_sensors = SimulatedSensors(self.dual_camera)

    self._exit_event = threading.Event()

    self.simulated_car_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_car.update, self.simulator_state),
                                                                        100, self._exit_event))
    self.simulated_car_thread.start()

    self.simulated_camera_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_sensors.send_camera_images, self.world),
                                                                        20, self._exit_event))
    self.simulated_camera_thread.start()

    # Simulation tends to be slow in the initial steps. This prevents lagging later
    for _ in range(20):
      self.world.tick()

    while self._keep_alive:
      throttle_out = steer_out = brake_out = 0.0
      throttle_op = steer_op = brake_op = 0.0

      # A key press is held for CRUISE_KEY_FRAMES loops, like a real button press (~100 ms). Held for one 10 ms loop,
      # the 100 Hz CAN thread (simulated_car) can sample around it and card never sees the press.
      self.simulator_state.cruise_button = self.cruise_key if self.cruise_key_frames > 0 else 0
      self.cruise_key_frames = max(self.cruise_key_frames - 1, 0)
      self.simulator_state.left_blinker = self.world.blinker > 0
      self.simulator_state.right_blinker = self.world.blinker < 0

      throttle_manual = steer_manual = brake_manual = 0.

      # Read manual controls
      if not q.empty():
        message = q.get()
        if message.type == QueueMessageType.CONTROL_COMMAND:
          m = message.info.split('_')
          if m[0] == "steer":
            steer_manual = float(m[1])
          elif m[0] == "throttle":
            throttle_manual = float(m[1])
          elif m[0] == "brake":
            brake_manual = float(m[1])
          elif m[0] == "cruise":
            self.cruise_key = {"down": CruiseButtons.DECEL_SET, "up": CruiseButtons.RES_ACCEL,
                               "cancel": CruiseButtons.CANCEL, "main": CruiseButtons.MAIN}.get(m[1], 0)
            self.cruise_key_frames = self.CRUISE_KEY_FRAMES
            self.simulator_state.cruise_button = self.cruise_key
          elif m[0] == "blinker":
            if m[1] == "left":
              self.simulator_state.left_blinker = True
            elif m[1] == "right":
              self.simulator_state.right_blinker = True
          elif m[0] == "ignition":
            self.simulator_state.ignition = not self.simulator_state.ignition
          elif m[0] == "reset":
            self.world.reset()
          elif m[0] == "quit":
            break

      self.simulator_state.user_brake = brake_manual
      self.simulator_state.user_gas = throttle_manual
      self.simulator_state.user_torque = steer_manual * -10000

      steer_manual = steer_manual * -40

      # Update openpilot on current sensor state
      self.simulated_sensors.update(self.simulator_state, self.world)

      # simulated_car_thread already runs self.simulated_car.sm.update() at 100 Hz (SimulatedCar.update). Updating it here
      # too read the same ZMQ sockets from two threads; on macOS (ZMQ backend, not msgq shared memory) that tripped
      # libzmq's "Bad address (src/fq.cpp:56)" assertion and killed the bridge mid-episode in about half the runs.
      self.simulator_state.is_engaged = self.simulated_car.sm['selfdriveState'].active

      if self.simulator_state.is_engaged:
        throttle_op = np.clip(self.simulated_car.sm['carControl'].actuators.accel / 1.6, 0.0, 1.0)
        brake_op = np.clip(-self.simulated_car.sm['carControl'].actuators.accel / 4.0, 0.0, 1.0)
        steer_op = self.simulated_car.sm['carControl'].actuators.steeringAngleDeg
        if self.steer_model is not None:
          v_ego = self.simulated_car.sm['carState'].vEgo
          self.steer_model.update(self.simulated_car.sm['carOutput'].actuatorsOutput.torque, v_ego)
          # Send MetaDrive the angle that gives the curvature this car's controller believes its steering-wheel angle
          # gives: the car's own VehicleModel (steer ratio and understeer), which is what latcontrol_torque measures
          # against (for the owner's Civic it is within 10% of kinematic tan(angle / steer ratio) / wheelbase).
          # MetaDrive's Bullet car does not steer kinematically: measured 0.00055 curvature per sent degree from 10 to
          # 30 deg at 8, 12 and 16 m/s, about 60% of kinematic for its 2.47 m wheelbase, with a dead band under 5 deg.
          if self.vehicle_model is None:
            self.vehicle_model = VehicleModel(self.simulated_car.sm['carParams'])
          curvature = self.vehicle_model.calc_curvature(math.radians(self.steer_model.angle), v_ego, 0.0)
          steer_op = curvature / METADRIVE_CURV_PER_DEG

        self.past_startup_engaged = True
        # Engaged at standstill, the planner holds the car stopped until the driver presses resume (as on the car).
        # Until the car first moves, press RES once a second, held like a real press. Before 2026-09-27 the set-speed
        # taps below did this by accident: they read a 2 Hz stale vCruise and kept tapping after engagement.
        if self.simulated_car.sm['carState'].vEgo > 1.0:
          self.moved_once = True
        if not self.moved_once and self.rk.frame % 100 == 0:
          self.cruise_key, self.cruise_key_frames = CruiseButtons.RES_ACCEL, self.CRUISE_KEY_FRAMES
        if self.simulated_car.sm['carState'].vCruise < self.target_cruise_kph - 1 and self.rk.frame % 30 == 0:
          self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL
      elif not self.past_startup_engaged and self.simulated_car.sm['selfdriveState'].engageable:
        self.simulator_state.cruise_button = CruiseButtons.DECEL_SET if self.startup_button_prev else CruiseButtons.MAIN # force engagement on startup
        self.startup_button_prev = not self.startup_button_prev

      throttle_out = throttle_op if self.simulator_state.is_engaged else throttle_manual
      brake_out = brake_op if self.simulator_state.is_engaged else brake_manual
      steer_out = steer_op if self.simulator_state.is_engaged else steer_manual

      # SIM_PLANT=civic: the Civic's own bicycle model moves the car (metadrive_process.CivicPlant); send it the
      # steering-wheel angle and the accel request instead of a MetaDrive steer/pedal
      if CIVIC_PLANT:
        if self.vehicle_model is None and self.simulated_car.sm['carParams'].mass > 0:
          self.vehicle_model = VehicleModel(self.simulated_car.sm['carParams'])
        if self.vehicle_model is not None:
          v_ego = self.simulated_car.sm['carState'].vEgo
          if self.simulator_state.is_engaged:
            wheel_deg = self.steer_model.angle if self.steer_model is not None else steer_op
            accel = self.simulated_car.sm['carControl'].actuators.accel
          else:
            wheel_deg = math.degrees(self.vehicle_model.get_steer_from_curvature(steer_manual * METADRIVE_CURV_PER_DEG, v_ego, 0.0))
            accel = throttle_manual * 1.6 - brake_manual * 4.0
          vm = self.vehicle_model
          self.world.plant_cmd = (wheel_deg, accel, (vm.m, vm.j, vm.l, vm.aF, vm.cF, vm.cR, vm.sR))

      self.world.apply_controls(steer_out, throttle_out, brake_out)
      self.world.read_state()
      self.world.read_sensors(self.simulator_state)
      if self.steer_model is not None:
        if not self.simulator_state.is_engaged:
          angle = 0.0
          if self.vehicle_model is not None:
            angle = self.vehicle_model.get_steer_from_curvature(steer_manual * METADRIVE_CURV_PER_DEG, self.simulated_car.sm['carState'].vEgo, 0.0)
          self.steer_model.reset(math.degrees(angle))
        self.simulator_state.steering_angle = self.steer_model.angle

      if self.world.exit_event.is_set():
        self.shutdown()

      if self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.world.tick()
        self.world.read_cameras()

      # don't print during test, so no print/IO Block between OP and metadrive processes
      if not self.test_run and self.rk.frame % 25 == 0:
        self.print_status()

      self.started.value = True

      self.rk.keep_time()
