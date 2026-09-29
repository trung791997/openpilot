import os
import traceback
import cereal.messaging as messaging

from cereal import car

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.can.dbc import DBC as DBCDefinition
from opendbc.car import Bus
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import DBC, HondaSafetyFlags
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.tools.sim.lib.common import SimulatorState


def load_car_config():
  """SIM_CAR_CONFIG (tools/sim/sim_car_config.py): a real car's logged CarParams, or None for the stock sim car."""
  config_dir = os.getenv("SIM_CAR_CONFIG")
  if not config_dir:
    return None
  with open(os.path.join(config_dir, "carParams.bin"), "rb") as f:
    return f.read()


class SimulatedCar:
  """Simulates a Honda Bosch car (panda state + can messages) to OpenPilot: a civic 2022 by default,
  or the car in SIM_CAR_CONFIG (same messages, packed with that car's DBC and safety param)."""

  def __init__(self):
    self.car_params_bytes = load_car_config()
    self.safety_param = HondaSafetyFlags.RADARLESS.value | HondaSafetyFlags.BOSCH_LONG.value
    dbc_name = "honda_bosch_radarless_generated"
    self.pt_bus = 0
    if self.car_params_bytes is not None:
      with car.CarParams.from_bytes(self.car_params_bytes) as cp:
        dbc_name = DBC[cp.carFingerprint][Bus.pt]
        self.safety_param = cp.safetyConfigs[-1].safetyParam
        # Bosch cars with a radar carry powertrain on bus 1 (the radar is on bus 0)
        self.pt_bus = CanBus(cp).pt
      # CarParamsCache is cleared on manager start; card reads it once the first CAN arrives, after this.
      Params().put("CarParamsCache", self.car_params_bytes)
    self.packer = CANPacker(dbc_name)
    self.pt_camera_packer = CANPacker(dbc_name)  # own rolling counter for the second copy of CAMERA_MESSAGES
    self.dbc_msgs = {m.name for m in DBCDefinition(dbc_name).msgs.values()}
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams', 'selfdriveState', 'carState', 'carOutput'])
    self.cp = CANParser(dbc_name, [], 0)
    self.idx = 0
    self.params = Params()
    self.obd_multiplexing = False

  def make_msg(self, name, bus, values):
    # a message the car's DBC lacks (the radarless CRUISE_FAULT_STATUS on older Bosch cars) is skipped
    # bus 0 below means the powertrain bus
    return self.packer.make_can_msg(name, self.pt_bus if bus == 0 else bus, values) if name in self.dbc_msgs else None

  def send_can_messages(self, simulator_state: SimulatorState):
    if not simulator_state.valid:
      return

    msg = []

    # *** powertrain bus ***

    speed = simulator_state.speed * 3.6 # convert m/s to kph
    msg.append(self.make_msg("ENGINE_DATA", 0, {"XMISSION_SPEED": speed}))
    msg.append(self.make_msg("WHEEL_SPEEDS", 0, {
      "WHEEL_SPEED_FL": speed,
      "WHEEL_SPEED_FR": speed,
      "WHEEL_SPEED_RL": speed,
      "WHEEL_SPEED_RR": speed
    }))

    msg.append(self.make_msg("SCM_BUTTONS", 0, {"CRUISE_BUTTONS": simulator_state.cruise_button}))

    msg.append(self.make_msg("GEARBOX_AUTO", 0, {"GEAR_SHIFTER": 4}))
    msg.append(self.make_msg("GAS_PEDAL_2", 0, {}))
    msg.append(self.make_msg("SEATBELT_STATUS", 0, {"SEATBELT_DRIVER_LATCHED": 1}))
    msg.append(self.make_msg("STEER_STATUS", 0, {"STEER_TORQUE_SENSOR": simulator_state.user_torque,
                                                     "STEER_STATUS": simulator_state.steer_status}))
    msg.append(self.make_msg("STEERING_SENSORS", 0, {"STEER_ANGLE": simulator_state.steering_angle}))
    msg.append(self.make_msg("VSA_STATUS", 0, {}))
    msg.append(self.make_msg("STANDSTILL", 0, {"WHEELS_MOVING": 1 if simulator_state.speed >= 1.0 else 0}))
    msg.append(self.make_msg("STEER_MOTOR_TORQUE", 0, {}))
    msg.append(self.make_msg("EPB_STATUS", 0, {}))
    msg.append(self.make_msg("DOORS_STATUS", 0, {}))
    msg.append(self.make_msg("CRUISE", 0, {}))
    msg.append(self.make_msg("CRUISE_FAULT_STATUS", 0, {}))
    msg.append(self.make_msg("SCM_FEEDBACK", 0,
                                    {
                                      "MAIN_ON": 1,
                                      "LEFT_BLINKER": simulator_state.left_blinker,
                                      "RIGHT_BLINKER": simulator_state.right_blinker
                                    }))
    msg.append(self.make_msg("POWERTRAIN_DATA", 0,
                                    {
                                    "ACC_STATUS": int(simulator_state.is_engaged),
                                    "PEDAL_GAS": simulator_state.user_gas,
                                    "BRAKE_PRESSED": simulator_state.user_brake > 0
                                    }))
    msg.append(self.make_msg("CAR_SPEED", 0, {}))

    # *** cam bus ***
    msg.append(self.make_msg("STEERING_CONTROL", 2, {}))
    msg.append(self.make_msg("ACC_HUD", 2, {}))
    msg.append(self.make_msg("LKAS_HUD", 2, {}))
    msg.append(self.make_msg("CAMERA_MESSAGES", 2, {}))
    if self.pt_bus != 0:
      # Bosch cars with a radar read the camera's sign messages from the powertrain bus
      msg.append(self.pt_camera_packer.make_can_msg("CAMERA_MESSAGES", self.pt_bus, {}))

    self.pm.send('can', can_list_to_can_capnp([m for m in msg if m is not None]))

  def send_panda_state(self, simulator_state):

    if self.params.get_bool("ObdMultiplexingEnabled") != self.obd_multiplexing:
      self.obd_multiplexing = not self.obd_multiplexing
      self.params.put_bool("ObdMultiplexingChanged", True)

    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': simulator_state.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'hondaBosch',
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
      'safetyParam': self.safety_param,
    }
    self.pm.send('pandaStates', dat)

  def update(self, simulator_state: SimulatorState):
    try:
      # Every loop: the bridge steers from sm['carOutput'] and engages from sm['selfdriveState']. Updated only in
      # send_panda_state (2 Hz) until 2026-09-27, the torque command reached the steering model up to 500 ms late
      # (250 ms on average), which put a ~1.8 s limit cycle of +/-40 deg into every torque-steered episode.
      self.sm.update(0)
      self.send_can_messages(simulator_state)

      if self.idx % 50 == 0: # only send panda states at 2hz
        self.send_panda_state(simulator_state)

      self.idx += 1
    except Exception:
      traceback.print_exc()
      raise
