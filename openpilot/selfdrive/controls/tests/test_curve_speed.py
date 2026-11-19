"""commaai/openpilot#39307 curve slowdown: CurveSpeed math and stock-planner-only wiring (static)."""
import inspect
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import curve_speed
from openpilot.selfdrive.controls.lib.curve_speed import CurveSpeed, MAX_ACCEL_OFFSET, MAX_OFFSET_JERK

DT = 0.05


def model(v, yaw_rate, t_max=10.0, n=33):
  t = [t_max * i / (n - 1) for i in range(n)]
  return SimpleNamespace(position=SimpleNamespace(t=t), velocity=SimpleNamespace(x=[v] * n),
                         orientationRate=SimpleNamespace(z=[yaw_rate] * n))


def test_straight_road_has_no_offset():
  cs = CurveSpeed(DT)
  for _ in range(100):
    assert cs.update(model(30.0, 0.0), 0.0) == 0.0


def test_tight_curve_ramps_to_max_offset_at_jerk_limit():
  cs = CurveSpeed(DT)
  out = [cs.update(model(30.0, 0.2), 0.0) for _ in range(40)]  # 6 m/s^2 lateral
  assert out[0] == pytest.approx(-MAX_OFFSET_JERK * DT)
  assert all(b - a >= -MAX_OFFSET_JERK * DT - 1e-9 for a, b in zip(out, out[1:], strict=False))
  assert out[-1] == pytest.approx(-MAX_ACCEL_OFFSET)


def test_curve_beyond_lookahead_is_ignored():
  cs = CurveSpeed(DT)
  m = model(30.0, 0.0)
  m.orientationRate.z = [0.2 if t >= curve_speed.LOOKAHEAD_TIME else 0.0 for t in m.position.t]
  assert cs.update(m, 0.0) == 0.0


def test_offset_releases_at_jerk_limit_and_reset_clears():
  cs = CurveSpeed(DT)
  for _ in range(40):
    cs.update(model(30.0, 0.2), 0.0)
  assert cs.update(model(30.0, 0.0), 0.0) == pytest.approx(-MAX_ACCEL_OFFSET + MAX_OFFSET_JERK * DT)
  cs.reset()
  assert cs.accel_offset == 0.0


def test_only_the_stock_planner_uses_it():
  from openpilot.starpilot.longitudinal.upstream import planner as stock
  from openpilot.selfdrive.controls.lib import longitudinal_planner as starpilot
  src = inspect.getsource(stock.LongitudinalPlanner.update)
  assert "output_a_target_e2e = sm['modelV2'].action.desiredAcceleration + curve_offset" in src
  assert 'self.curve_speed.reset()' in src
  assert 'CurveSpeed' not in inspect.getsource(starpilot)
