"""The live plots sampler must read engagement and the planner target from the services
that actually carry them. controlsState.active and controlsState.aTarget do not exist in
this schema, so reading them there returned defaults: the rating counted disengaged
driving and the longitudinal "desired" trace was the PID output, not the plan."""
from cereal import car, log

import starpilot.system.the_galaxy.the_galaxy as galaxy


class _FakeSubMaster:
  def __init__(self, messages):
    self._messages = messages
    self.recv_frame = {name: (1 if name in messages else 0) for name in
                       ("controlsState", "livePose", "carControl", "longitudinalPlan")}

  def update(self, timeout=0):
    pass

  def __getitem__(self, name):
    defaults = {
      "controlsState": log.ControlsState.new_message(),
      "livePose": log.LivePose.new_message(),
      "carControl": car.CarControl.new_message(),
      "longitudinalPlan": log.LongitudinalPlan.new_message(),
    }
    return self._messages.get(name, defaults[name])


def _run_one_sample(monkeypatch, messages):
  monkeypatch.setattr(galaxy.messaging, "SubMaster", lambda *a, **k: _FakeSubMaster(messages))

  def _stop_after_one(_):
    galaxy._plots_last_client_request_ts = 0.0

  monkeypatch.setattr(galaxy.time, "sleep", _stop_after_one)
  monkeypatch.setattr(galaxy, "_is_plots_boot_stabilizing", lambda: False)
  galaxy._plots_last_client_request_ts = galaxy.time.monotonic()
  galaxy._plots_worker()
  with galaxy._plots_lock:
    return dict(galaxy._plots_state)


def _controls_state_with_pid_output(total):
  cs = log.ControlsState.new_message()
  cs.upAccelCmd = total
  return cs


def test_engaged_flags_come_from_car_control(monkeypatch):
  cc = car.CarControl.new_message()
  cc.latActive = True
  cc.longActive = False  # e.g. gas override
  state = _run_one_sample(monkeypatch, {"carControl": cc})
  assert state["lastError"] == ""
  assert state["controlsActive"] is True
  assert state["longitudinalControlActive"] is False


def test_not_engaged_without_car_control(monkeypatch):
  state = _run_one_sample(monkeypatch, {})
  assert state["controlsActive"] is False
  assert state["longitudinalControlActive"] is False


def test_desired_accel_is_plan_target_not_pid_output(monkeypatch):
  plan = log.LongitudinalPlan.new_message()
  plan.aTarget = -1.25
  state = _run_one_sample(monkeypatch, {"longitudinalPlan": plan,
                                         "controlsState": _controls_state_with_pid_output(0.4)})
  assert state["desiredLongitudinalAccel"] == -1.25
  assert state["longitudinalSource"].startswith("longitudinalPlan.aTarget")


def test_zero_plan_target_is_not_replaced_by_pid_output(monkeypatch):
  plan = log.LongitudinalPlan.new_message()
  plan.aTarget = 0.0
  state = _run_one_sample(monkeypatch, {"longitudinalPlan": plan,
                                         "controlsState": _controls_state_with_pid_output(0.4)})
  assert state["desiredLongitudinalAccel"] == 0.0


def test_pid_fallback_only_without_a_plan(monkeypatch):
  state = _run_one_sample(monkeypatch, {"controlsState": _controls_state_with_pid_output(0.4)})
  assert abs(state["desiredLongitudinalAccel"] - 0.4) < 1e-4
  assert "PID sum" in state["longitudinalSource"]
