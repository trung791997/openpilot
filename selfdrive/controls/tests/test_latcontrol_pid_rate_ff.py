from types import SimpleNamespace

import pytest

from cereal import car, custom, log
import openpilot.selfdrive.controls.lib.latcontrol_pid as latcontrol_pid
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA, HondaFlags
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID


class _Params:
  def __init__(self, values):
    self.values = values

  def get(self, key, *args, **kwargs):
    return self.values.get(key)

  def get_bool(self, key, *args, **kwargs):
    return str(self.values[key]) in ("1", "True", "true")   # KeyError -> the controller's default


def _build(monkeypatch, values):
  monkeypatch.setattr(latcontrol_pid, "civic_bosch_modified_lateral_testing_ground_active", lambda *a, **kw: False)
  monkeypatch.setattr(latcontrol_pid, "Params", lambda: _Params(values))
  CarInterface = interfaces[HONDA.HONDA_CIVIC_BOSCH]
  CP = CarInterface.get_non_essential_params(HONDA.HONDA_CIVIC_BOSCH)
  CP.flags |= int(HondaFlags.EPS_MODIFIED)
  CP.dashcamOnly = True
  CI = CarInterface(CP, custom.StarPilotCarParams.new_message())
  lac = LatControlPID(CP.as_reader(), CI, DT_CTRL)
  lac.params = _Params(values)
  params = log.LiveParametersData.new_message()
  params.steerRatio = CP.steerRatio
  params.stiffnessFactor = 1.0
  params.angleOffsetDeg = 0.0
  return lac, VehicleModel(CP), params


def _run(monkeypatch, values, frames=400):
  """A steady turn-in from frame 320. The wheel follows the shaped target one frame late, so the PID
  terms stay small and the output does not saturate."""
  lac, VM, params = _build(monkeypatch, values)
  CS = car.CarState.new_message()
  CS.vEgo = 15.0
  CS.steeringAngleDeg = 0.0
  outs, raws, shaped = [], [], []
  for k in range(frames):
    curv = 0.00002 * max(k - 320, 0)
    out, des, _ = lac.update(True, CS, VM, params, False, curv, False, 0.2, None, None, SimpleNamespace())
    CS.steeringAngleDeg = des
    outs.append(out)
    raws.append(lac.raw_angle_steers_des)
    shaped.append(des)
  return lac, outs, raws, shaped


def test_rate_ff_defaults_off_and_reads_its_param(monkeypatch):
  lac, *_ = _run(monkeypatch, {})
  assert lac.rate_ff == 0.0
  lac, *_ = _run(monkeypatch, {"NrdrLatRateFF": "0.5"})
  assert lac.rate_ff == pytest.approx(0.5)


def test_rate_ff_adds_torque_in_the_direction_of_target_slew(monkeypatch):
  _, off, _, shaped = _run(monkeypatch, {})
  _, on, _, _ = _run(monkeypatch, {"NrdrLatRateFF": "0.5"})
  assert on[:320] == pytest.approx(off[:320])   # target still: nothing added
  slew = (shaped[-1] - shaped[-2]) / DT_CTRL
  assert abs(slew) > 5.0
  extra = on[-1] - off[-1]
  assert extra * slew > 0
  assert abs(extra) > 0.2 * 0.5 * abs(slew) / 100.0   # the output scales can shrink it, not remove it


def test_raw_target_is_the_one_before_shaping(monkeypatch):
  _, _, raw, shaped = _run(monkeypatch, {})
  assert abs(raw[-1]) > abs(shaped[-1])   # the smoothing filter trails a ramping target
  assert raw[100] == pytest.approx(shaped[100])
