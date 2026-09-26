from types import SimpleNamespace

from cereal import car
from openpilot.selfdrive.controls.tests.test_latcontrol_pid_rate_ff import _build

MPH = 0.44704


def _step(lac, VM, params, CS, limited=False, active=True):
  lac.update(active, CS, VM, params, limited, 0.0, False, 0.2, None, None, SimpleNamespace())


def _wound_up(monkeypatch, values=None):
  """20 mph, target straight ahead, wheel held 0.2 deg right for 3 s: a live integrator into the error."""
  lac, VM, params = _build(monkeypatch, values or {})
  CS = car.CarState.new_message()
  CS.vEgo = 20.0 * MPH
  CS.steeringAngleDeg = -0.2
  for _ in range(300):
    _step(lac, VM, params, CS)
  assert lac.pid.i > 0.02
  return lac, VM, params, CS


def _press(lac, VM, params, CS, frames=50):
  CS.steeringPressed = True
  CS.steeringTorque = 3000.0
  for _ in range(frames):
    _step(lac, VM, params, CS, limited=True)
  CS.steeringPressed = False
  CS.steeringTorque = 0.0


def test_disengage_clears_the_integrator(monkeypatch):
  lac, VM, params, CS = _wound_up(monkeypatch)
  _step(lac, VM, params, CS, active=False)
  assert lac.pid.i == 0.0


def test_reset_clears_the_integrator(monkeypatch):
  lac, *_ = _wound_up(monkeypatch)
  lac.reset()
  assert lac.pid.i == 0.0 and lac.sat_time == 0.0


def test_integrator_bleeds_through_the_override_fade(monkeypatch):
  lac, VM, params, CS = _wound_up(monkeypatch, {"HondaOverrideFadeUpSecs": "0.5"})
  assert lac.override_fade_up_s == 0.5
  _press(lac, VM, params, CS)
  held = lac.pid.i
  assert held > 0.02            # frozen, not bled, while the driver holds the wheel
  for _ in range(50):
    _step(lac, VM, params, CS, limited=True)
  assert 0.0 < lac.pid.i < held * 0.45   # a bleed, not a reset
  after = lac.pid.i
  for _ in range(100):          # past the fade: a limit with no recent press still just freezes
    _step(lac, VM, params, CS, limited=True)
  assert lac.pid.i == after


def test_limit_without_a_press_still_freezes(monkeypatch):
  lac, VM, params, CS = _wound_up(monkeypatch)
  held = lac.pid.i
  for _ in range(100):
    _step(lac, VM, params, CS, limited=True)
  assert lac.pid.i == held
