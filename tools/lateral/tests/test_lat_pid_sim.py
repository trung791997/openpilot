import numpy as np
import pytest

from openpilot.tools.lateral import lat_pid_sim as sim


def _windows(n=40, win=sim.PLANT_WIN, seed=0):
  rng = np.random.default_rng(seed)
  pre = max(sim.FIT_DELAYS)
  u = np.repeat(rng.uniform(-0.3, 0.3, (n, (pre + win + 1) // 10 + 1)), 10, axis=1)[:, :pre + win + 1]
  return {"v": np.full((n, win + 1), 15.0), "u": u, "th": np.zeros((n, win + 1)), "r": np.zeros((n, win + 1))}


def test_delayed_plant_answers_the_command_delay_frames_late():
  p = _windows()
  coef = [-8.0, 0.0, -2.0, 0.0, 300.0, 0.0, 0.0, 0.0, 0.0]
  now = sim._plant_freerun(p, sim.Plant(coef, 0))
  late = sim._plant_freerun(p, sim.Plant(coef, 6))
  shifted = dict(p, u=np.roll(p["u"], 6, axis=1))
  assert np.allclose(late, sim._plant_freerun(shifted, sim.Plant(coef, 0)))
  assert not np.allclose(now, late)


def test_fit_recovers_a_synthetic_delay():
  p = _windows(n=60)
  truth = sim.Plant([-8.0, 0.0, -2.0, 0.0, 300.0, 0.0, 0.0, 0.0, 0.0], 8)
  p["th"] = sim._plant_freerun(p, truth)
  costs = {dl: sim._fit_coef(p, dl, 30, False)[1] for dl in (0, 4, 8, 12)}
  assert min(costs, key=costs.get) == 8


def test_plant_json_round_trip_and_old_files_stay_undelayed():
  pl = sim.Plant([1.0] * 9, 5, 0.1)
  back = sim.Plant.from_json(pl.to_json())
  assert (back.delay, back.quant) == (5, 0.1)
  old = sim.Plant.from_json({"coef": [1.0] * 9})
  assert (old.delay, old.quant) == (0, 0.0)
  assert old.measure(0.123) == 0.123 and back.measure(0.123) == pytest.approx(0.1)


def test_sign_changes_use_the_analyzer_deadband():
  from openpilot.selfdrive.controls.lib import lat_tune_analyzer as lat
  assert sim.SIGN_HYST_DEG == lat.SIGN_HYST_DEG
  err = np.array([0.1, -0.1] * 50 + [0.3, -0.3] * 50)
  assert sim.sign_changes(err, np.ones(len(err), bool)) == 99
  straight = np.ones(len(err), bool)
  straight[150] = False   # a non-straight frame restarts the count, as DriveStats.observe does
  assert sim.sign_changes(err, straight) == 97   # the changes into and out of frame 150 are both lost


def test_tracking_lag_finds_the_shift():
  t = np.arange(3000) * sim.DT
  des = 10 * np.sin(2 * np.pi * 0.2 * t)
  ang = np.r_[np.zeros(20), des[:-20]]
  assert sim.tracking_lag(des, ang, np.abs(des) > 5) == pytest.approx(0.20)
