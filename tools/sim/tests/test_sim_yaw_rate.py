"""The sim's KINEMATICS yaw rate, packed with the Civic Bosch DBC and decoded the way carstate does (yaw_rate.py),
reads back as the plant's yaw rate in rad/s, left-positive. No sim is run."""
import math

import pytest

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.honda.values import CAR, DBC
from opendbc.car.honda.yaw_rate import get_yaw_rate_calibration
from openpilot.tools.sim.lib.simulated_car import kinematics_yaw_rate

COUNT_RAD_S = math.radians(0.244)  # one sensor count


def car_yaw_rate(yaw_rate: float) -> float:
  dbc = DBC[CAR.HONDA_CIVIC_BOSCH][Bus.pt]
  packer, parser = CANPacker(dbc), CANParser(dbc, [("KINEMATICS", 100)], 0)
  addr, dat, bus = packer.make_can_msg("KINEMATICS", 0, {"YAW_RATE": kinematics_yaw_rate(yaw_rate)})
  parser.update([(0, [(addr, dat, bus)])])
  return -get_yaw_rate_calibration(CAR.HONDA_CIVIC_BOSCH).update(parser.vl["KINEMATICS"]["YAW_RATE"], False) * math.pi / 180


@pytest.mark.parametrize("yaw_rate", [0.0, 0.05, -0.05, 0.3, -0.3, 1.2, -1.2])
def test_round_trip(yaw_rate):
  assert car_yaw_rate(yaw_rate) == pytest.approx(yaw_rate, abs=COUNT_RAD_S / 2 + 1e-9)


def test_left_turn_is_positive_and_clockwise_in_dbc():
  assert kinematics_yaw_rate(0.2) < 0  # the DBC signal is clockwise-positive
  assert car_yaw_rate(0.2) > 0


def test_saturates_at_sensor_range():
  # 10 bits: a hard left pins the count at 0, a hard right at 1023, about the 513 zero
  assert car_yaw_rate(10.0) == pytest.approx(513 * COUNT_RAD_S)
  assert car_yaw_rate(-10.0) == pytest.approx(-(1023 - 513) * COUNT_RAD_S)
