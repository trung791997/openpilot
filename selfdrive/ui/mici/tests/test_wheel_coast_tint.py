import pytest

from openpilot.selfdrive.ui.onroad.exp_button import (ACCEL_WHEEL_COLOR, BRAKE_WHEEL_COLOR, COAST_WHEEL_COLOR,
                                                      get_wheel_tint)

MODE = object()


@pytest.mark.parametrize("kw,expected", [
  (dict(coasting=True, commanded_accel=-0.3, acceleration=-0.3), COAST_WHEEL_COLOR),
  (dict(coasting=True, commanded_accel=-0.3, brake_lights=True), BRAKE_WHEEL_COLOR),
  (dict(coasting=True, brake_pressed=True), BRAKE_WHEEL_COLOR),
  (dict(coasting=True, gas_pressed=True), ACCEL_WHEEL_COLOR),
  (dict(coasting=False, commanded_accel=-0.3), BRAKE_WHEEL_COLOR),
  (dict(coasting=False, commanded_accel=0.3), ACCEL_WHEEL_COLOR),
])
def test_coast_tint(kw, expected):
  kw = dict(kw)
  brake_pressed = kw.pop("brake_pressed", False)
  assert get_wheel_tint(brake_pressed, MODE, True, **kw) == expected


def test_feedback_off_keeps_mode_tint():
  assert get_wheel_tint(False, MODE, False, coasting=True) is MODE
