from types import SimpleNamespace

import numpy as np
from cereal import car

import openpilot.selfdrive.controls.lib.latcontrol_pid as latcontrol_pid
from openpilot.selfdrive.controls.tests.test_latcontrol_pid_rate_ff import _build

MPH = 0.44704


def _cross_50mph(monkeypatch, values, above=900, below=300):
  """Route 277 in miniature: a held-off wheel keeps a small steady error while the car sits just above
  50 mph (I trim = LatIScaleHighway), then slows to 49.98 mph (I trim = LatIScaleStandard)."""
  lac, VM, params = _build(monkeypatch, values)
  CS = car.CarState.new_message()
  CS.steeringAngleDeg = -0.3   # target straight ahead, wheel held 0.3 deg right
  outs, iis = [], []
  for k in range(above + below):
    CS.vEgo = 50.05 * MPH if k < above else 49.98 * MPH
    out, _, _ = lac.update(True, CS, VM, params, False, 0.0, False, 0.2, None, None, SimpleNamespace())
    outs.append(out)
    iis.append(lac.pid.i)
  return lac, np.array(outs), np.array(iis), above


def test_crossing_the_50mph_band_edge_does_not_step_the_output(monkeypatch):
  _, outs, iis, edge = _cross_50mph(monkeypatch, {"LatIScaleStandard": "75", "LatIScaleHighway": "0"})
  assert abs(iis[edge - 1]) < 0.02   # no hidden integrator built up while the I trim was 0
  steps = np.abs(np.diff(outs[edge - 5:]))
  assert steps.max() < 0.01


def test_band_changes_slew_instead_of_stepping(monkeypatch):
  lac, outs, _, edge = _cross_50mph(monkeypatch, {"LatIScaleStandard": "75", "LatIScaleHighway": "0",
                                                  "LatFScaleStandard": "50", "LatFScaleHighway": "100"})
  _, i_applied, f_applied = lac.applied_scales
  assert 0.0 < i_applied <= 300 * latcontrol_pid.NRDR_TRIM_SLEW_PER_S * lac.dt + 1e-9
  assert f_applied == 0.5   # 0.5 away, reached at 0.5/s in 1 s
  assert np.abs(np.diff(outs[edge - 5:])).max() < 0.01
