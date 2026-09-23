from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.redneck_cruise import (
  DECREASE_INACTIVE_TIMER,
  INCREASE_INACTIVE_TIMER,
  RedneckCruise,
  SEND_BUTTON_DECREASE,
  SEND_BUTTON_NONE,
)

FRAMES = int(max(INCREASE_INACTIVE_TIMER, DECREASE_INACTIVE_TIMER) / DT_CTRL) + 2


def _run(redneck, gas_pressed, target_mph=30.0, cluster_mph=40.0, frames=FRAMES):
  send_button = SEND_BUTTON_NONE
  for _ in range(frames):
    CS = SimpleNamespace(cruiseState=SimpleNamespace(speedCluster=cluster_mph * CV.MPH_TO_MS), buttonEvents=[],
                         gasPressed=gas_pressed)
    CC = SimpleNamespace(enabled=True, cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False))
    send_button, _ = redneck.run(CS, CC, target_mph * CV.MPH_TO_MS, is_metric=False)
  return send_button


def _redneck():
  return RedneckCruise(SimpleNamespace(), SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True))


def test_holds_presses_while_driver_is_on_the_gas():
  # Stock ACC never sets cruiseControl.override; a DECEL/SET under gas snaps the set speed to vEgo.
  assert _run(_redneck(), gas_pressed=True) == SEND_BUTTON_NONE


def test_resumes_after_gas_release():
  redneck = _redneck()
  assert _run(redneck, gas_pressed=True) == SEND_BUTTON_NONE
  assert _run(redneck, gas_pressed=False) == SEND_BUTTON_DECREASE
