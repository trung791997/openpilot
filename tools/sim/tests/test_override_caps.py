"""Static tests for the sim's driver-override capabilities: the EPS fault model (eps_status.py), the driver hand's gap kind,
torque cap, release ramp and re-press (driver_model.py), and the twin-spread split (twin_align.py). No sim is run."""
import json

import numpy as np

from openpilot.tools.sim.lib.driver_model import DriverModel, window
from openpilot.tools.sim.lib.eps_status import GAP_FRAMES, NO_TORQUE_ALERT_1, NORMAL, ON_FRAMES, TEMPLATES, EpsStatus
from openpilot.tools.sim.override_score import SCEN, want_trace
from openpilot.tools.sim.twin_align import align, load


def run_eps(spec, tq, v=2.0, press_at=None):
  e = EpsStatus(spec)
  return e, [e.update(x, v, press_at is not None and k >= press_at) for k, x in enumerate(tq)]


def test_eps_off_below_threshold():
  _, st = run_eps({"threshold": True}, [3100.0] * 500)
  assert set(st) == {NORMAL}


def test_eps_threshold_flickers_and_repeats():
  tq = [3400.0] * 1000
  e1, s1 = run_eps({"threshold": True, "seed": 3}, tq)
  e2, s2 = run_eps({"threshold": True, "seed": 3}, tq)
  assert s1 == s2  # the same torque trace flickers the same way (twin runs)
  assert s1[0] == NO_TORQUE_ALERT_1
  assert len(e1.log) > 1  # the EPS clears and re-raises it while the torque stays high
  gaps = [b[0] - a[1] for a, b in zip(e1.log[:-1], e1.log[1:], strict=True)]
  assert min(gaps) >= GAP_FRAMES[0] * 0.01 - 0.011 and max(gaps) <= GAP_FRAMES[-1] * 0.01 + 0.011


def test_eps_on_lengths_follow_road():
  e, _ = run_eps({"threshold": True, "seed": 1}, [3400.0] * 200000)
  on = np.array([b - a for a, b in e.log if b is not None])
  road = np.array(ON_FRAMES) * 0.01
  for q in (50, 90):  # draws come straight from the EPS-ended road runs
    lo, hi = np.percentile(road, q, method="lower"), np.percentile(road, q, method="higher")
    assert lo - 0.011 <= np.percentile(on, q, method="lower") <= hi + 0.011


def test_eps_hysteresis_and_speed_gate():
  _, st = run_eps({"threshold": True}, [3300.0] * 5 + [3160.0] * 5 + [3100.0] * 5)
  assert st[0] == NO_TORQUE_ALERT_1 and st[-1] == NORMAL
  _, st = run_eps({"threshold": True, "v_max": 4.5}, [3400.0] * 100, v=5.0)
  assert set(st) == {NORMAL}


def test_eps_template_timing():
  e, st = run_eps({"events": [{"template": "294s14"}]}, [0.0] * 800, press_at=100)
  tpl = TEMPLATES["294s14"]
  assert np.allclose([b - a for a, b in e.log], tpl["on"], atol=0.011)
  assert abs(e.log[0][0] - 1.01 - tpl["after_press_s"]) <= 0.011  # press flag first seen on step 101 (t 1.01 s); one 10 ms step
  assert st[-1] == NORMAL


def test_eps_unset_is_normal():
  _, st = run_eps({}, [5000.0] * 200)
  assert set(st) == {NORMAL}


def test_gap_never_past_straight():
  d = DriverModel({"presses": [{"kind": "gap", "gap_deg": 50, "at_s": 0.0, "dur": 5.0, "r": 0.3}]})
  d.update(30.0, 30.0, 0.0, 5.0)
  d.t = 2.0
  assert d.want(30.0) == 0.0 and d.want(-30.0) == 0.0
  assert d.want(100.0) == 50.0 and d.want(-100.0) == -50.0


def test_tq_cap():
  for spec in ({"tq_cap": 1750}, {}):
    d = DriverModel(dict(spec, ki="firm", presses=[{"kind": "gap", "gap_deg": 50, "at_s": 0.0, "dur": 5.0, "r": 0.1}]))
    tq = [d.update(120.0, 120.0, 0.0, 5.0) for _ in range(400)]  # the wheel never moves: the hand saturates
    assert max(abs(x) for x in tq) <= spec.get("tq_cap", 3500.0) + 1e-6
    assert max(abs(x) for x in tq) > 0.95 * spec.get("tq_cap", 3500.0)


def test_road_release_ramp():
  d = DriverModel({"presses": [{"kind": "offset", "offset_deg": 5, "at_s": 0.0, "dur": 3.0, "r": 0.4, "r_out": "road"}]})
  d.update(20.0, 20.0, 0.0, 2.0)
  assert abs(d.r_out - 0.18) < 1e-9
  p, t0, t1 = d.active
  assert abs(window(t1 - 0.09, t0, t1, 0.4, d.r_out) - 0.5) < 1e-9
  d = DriverModel({"presses": [{"kind": "offset", "offset_deg": 5, "at_s": 0.0, "dur": 3.0, "r": 0.4, "r_out": "road"}]})
  d.update(20.0, 20.0, 0.0, 20.0)
  assert abs(d.r_out - 0.45) < 1e-9


def repress_run(snap_step):
  d = DriverModel({"presses": [{"kind": "offset", "offset_deg": 5, "at_s": 0.0, "dur": 1.0, "r": 0.2,
                                "repress": {"snap_deg": 3.0, "after": [0.3, 0.8], "dur": 1.5}}]})
  ang = 15.0
  for k in range(300):
    if k == 150:
      ang += snap_step  # 0.49 s after release: the wheel springs back toward the plan
    d.update(20.0, ang, 0.0, 8.0)
  return d


def test_repress():
  d = repress_run(4.0)
  assert len(d.log) == 2 and abs(d.log[1][1] - d.log[1][0] - 1.5) < 1e-9
  assert 0.3 <= d.log[1][0] - d.log[0][1] <= 0.8
  assert len(repress_run(1.0).log) == 1


def test_default_spec_unchanged_by_new_options():
  spec = {"version": "v2", "ki": 800, "presses": [{"kind": "offset", "offset_deg": 3, "trigger_deg": 8, "dwell": 1.0, "dur": 4.0, "r": 0.4}]}
  a, b = DriverModel(spec), DriverModel(json.loads(json.dumps(spec)) | {"tq_cap": 3500.0})
  plan = 12.0 * np.sin(np.arange(1500) * 0.004)
  ta = [a.update(p, 0.9 * p, 1.0, 20.0) for p in plan]
  tb = [b.update(p, 0.9 * p, 1.0, 20.0) for p in plan]
  assert ta == tb and a.log == b.log


def test_scenarios_parse():
  for name in ("gapturn_12_25", "gapturn_12_50", "gapturn_16_25", "gapturn_16_50", "gapturn_12_50_cut", "gapturn_12_50_fault",
               "flicker_12_294s14", "repress_30", "handsoff_16"):
    s = SCEN[name]
    if s["press"] is not None:
      DriverModel({"presses": [s["press"]]})
    if "eps_status" in s:
      EpsStatus(s["eps_status"])
  t = np.arange(0.0, 6.0, 0.01)
  w = want_trace(SCEN["gapturn_12_50"]["press"], t, np.full_like(t, 90.0), 5.3)
  assert abs(w[250] - 40.0) < 1e-9 and w[-1] == 90.0


def write_run(path, delay, lat_at_trigger=0.0):
  (path / "frames").mkdir(parents=True)
  t = np.arange(0.0, 20.0, 0.05)
  v = np.full_like(t, 10.0)
  plan = np.where(t >= 5.0 + delay, 12.0, 0.0)  # hug_45 triggers 1.5 s after |plan| >= 9.6
  tr = 6.5 + delay
  lat = lat_at_trigger + np.where(t > tr, 0.2 * (t - tr), 0.0)
  with open(path / "frames" / "lane_gt.csv", "w") as f:
    f.write("t_mono,frame,speed,lane_idx,long_m,lat_m,heading_err,yaw_rate,lane_curv\n")
    for k in range(len(t)):
      f.write(f"{t[k]:.4f},{5 * k},{v[k]:.3f},0,0,{lat[k]:.4f},0,0,0\n")
  np.savez(path / "lat_pid_sim.npz", des_angle=plan, v=v)
  np.savez(path / "lanes.npz", t_mono=t)


def test_twin_align_splits_timing_from_path(tmp_path):
  press = SCEN["hug_45"]["press"]
  for i, dl in enumerate((0.0, 0.3)):
    write_run(tmp_path / f"r{i}", dl)
  runs = [load(str(tmp_path / f"r{i}"), press) for i in range(2)]
  out = align(runs, press["dur"])
  assert out["rel3s"]["time_m"] < 0.02  # a pure delay of the whole run: the trigger follows it
  write_run(tmp_path / "r2", 0.0, lat_at_trigger=0.5)
  out = align([runs[0], load(str(tmp_path / "r2"), press)], press["dur"])
  assert abs(out["rel3s"]["time_m"] - 0.5) < 0.02 and out["rel3s"]["anchored_m"] < 0.02
