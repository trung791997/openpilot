#!/usr/bin/env python3
"""Score a MetaDrive driver-override episode (SIM_DRIVER, tools/sim/lib/driver_model.py) with Driver override (Kevin)'s
metric set (2026-09-28), so the MetaDrive runs and his offline override sim report the same numbers.

  tools/sim/override_score.py RUN_DIR [RUN_DIR ...]

The press start comes from the bridge log ("sim_driver: press N start <time.monotonic()>"), matched to the recorder's
t_mono (lanes.npz); the press shape (kind, offset/scale, dur, r) from the episode's SIM_DRIVER in override_scenarios.json
(scenario = the run name's ovr_<scenario>_ part). Windows: hold = [t0, t1), late = [t1 - 1.5, t1), after = [t1, t1 + 3).
  achieved_frac = mean(ang - plan) / mean(want - plan) over late        driver_tq mean / peak = |carState.steeringTorque|
  resist = hold & delivered * sign(want - plan) < -0.02 (s, peak, integral)   (delivered = carOutput torque, -1..1)
  cut = rising edges of carState.steeringPressed within hold, and seconds pressed (the carcontroller latch is not logged)
  release: overshoot past the plan, peak |rate|, back_on_plan_s (|ang - plan| < 1 deg for 0.5 s; 3.0 if never)
  after release (lane_gt.csv, to 6 s): post_lat_peak_m from lane centre, post_back_0p3m_s, post_hdg_peak_deg
blip*: blip_step_max = max |per-frame change in delivered| over [blip - 0.1, blip + 0.6) (blip_step_before: the second before)
push_* scenarios (no hand loop): worst wheel change over the last 2 s of the push. Sim evidence only.
"""
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from openpilot.tools.sim.lib.driver_model import ROAD_R_OUT, ROAD_R_OUT_BP, window  # noqa: E402

with open(os.path.join(os.path.dirname(__file__), "maps", "override_scenarios.json")) as f:
  SCEN = json.load(f)


def press_start(run):
  """(scenario, press spec, press start in time.monotonic() or None)."""
  name = os.path.basename(run.rstrip("/"))
  scen = re.match(r"ovr_(.+)_\d+_\d+_[^_]+$", name)[1]
  starts = re.findall(r"sim_driver: press \d+ start ([\d.]+)", open(f"{run}/bridge.log", errors="ignore").read())
  return scen, SCEN[scen]["press"], (float(starts[0]) if starts else None)


def want_trace(press, t, plan, v0=None):
  """v0: speed at press start (m/s), for r_out "road" (driver_model.ROAD_R_OUT); without it the ramp-in r is used."""
  if press is None or press["kind"] not in ("offset", "widen", "scale", "gap"):
    return None
  r_out = press.get("r_out")
  if r_out == "road":
    r_out = float(np.interp(v0, ROAD_R_OUT_BP, ROAD_R_OUT)) if v0 is not None else None
  w = np.array([window(x, 0.0, press["dur"], press.get("r", 0.4), r_out) for x in t])
  if press["kind"] == "offset":
    return plan - np.copysign(press["offset_deg"], plan) * w
  if press["kind"] == "gap":
    return plan - np.copysign(np.minimum(press["gap_deg"], np.abs(plan)), plan) * w
  if press["kind"] == "widen":
    return plan + np.copysign(press["offset_deg"], plan) * w
  return plan * (1 - press["scale"] * w)


def score(run):
  name = os.path.basename(run.rstrip("/"))
  scen = re.match(r"ovr_(.+)_\d+_\d+_[^_]+$", name)[1]
  press = SCEN[scen]["press"]
  z = np.load(f"{run}/lat_pid_sim.npz")
  tm = np.load(f"{run}/lanes.npz")["t_mono"]
  ang, plan, deliv, tq, pressed = z["angle"], z["des_angle"], z["co_torque"], z["eps_torque"], z["pressed"] > 0
  starts = [float(m) for m in re.findall(r"sim_driver: press \d+ start ([\d.]+)", open(f"{run}/bridge.log", errors="ignore").read())]
  out = {"run": name}
  if SCEN[scen].get("blips") and len(starts) > (1 if press else 0):
    # raw sensor blip (driver_model "blips"; the bridge prints it as the next press): delivered-torque steps around it
    tb = tm - starts[1 if press else 0]
    wb = (tb >= -0.1) & (tb < 0.6)
    step = np.abs(np.diff(deliv))
    out["blip_step_max"] = float(step[wb[1:]].max())
    out["blip_step_before"] = float(step[((tb >= -1.1) & (tb < -0.1))[1:]].max())
    out["blip_pressed_frames"] = int(pressed[wb].sum())
  if press is None:
    out["note"] = "hands-off"
    out["max_abs_ang_minus_plan"] = float(np.max(np.abs(ang - plan)[z["active"] > 0]))
    return out
  if not starts:
    out["note"] = "press never started"
    return out
  t = tm - starts[0]
  dur = press["dur"]
  hold = (t >= 0) & (t < dur)
  late = (t >= dur - 1.5) & (t < dur)
  after = (t >= dur) & (t < dur + 3)
  if press["kind"] == "push":
    last = (t >= dur - 2) & (t < dur)
    out["worst_wheel_change_deg"] = float(np.ptp(ang[last]))
    out["mean_ang_last2s"] = float(np.mean(ang[last]))
    out["driver_tq_peak"] = float(np.max(np.abs(tq[hold])))
    return out
  want = want_trace(press, t, plan, float(np.interp(0.0, t, z["v"])))
  if want is None:  # rest: no want; report excursions
    out["max_abs_ang_minus_plan"] = float(np.max(np.abs(ang - plan)[hold]))
    out["driver_tq_peak"] = float(np.max(np.abs(tq[hold])))
    return out
  want_off = float(np.mean((want - plan)[late]))
  d = np.sign(want_off)
  out["achieved_frac"] = float(np.mean((ang - plan)[late]) / want_off)
  out["driver_tq_mean"] = float(np.mean(np.abs(tq[hold])))
  out["driver_tq_peak"] = float(np.max(np.abs(tq[hold])))
  opp = hold & (deliv * d < -0.02)
  dt = float(np.median(np.diff(tm)))
  out["resist_s"] = float(opp.sum() * dt)
  out["resist_peak"] = float(np.max(np.abs(deliv[opp]))) if opp.any() else 0.0
  out["resist_int"] = float(np.sum(np.abs(deliv[opp])) * dt)
  ph = pressed & hold
  out["raw_press_edges"] = int(np.sum(ph[1:] & ~ph[:-1]) + (1 if ph[np.argmax(hold)] else 0))
  out["raw_press_s"] = float(ph.sum() * dt)
  e = ang - plan
  out["release_overshoot_deg"] = float(np.max(-d * e[after])) if after.any() else float("nan")
  rate = np.gradient(ang, tm)  # the npz rate column is not filled by the recorder
  out["release_peak_rate_dps"] = float(np.max(np.abs(rate[after]))) if after.any() else float("nan")
  ia = np.where(after)[0]
  ok = np.abs(e[ia]) < 1.0
  n = int(round(0.5 / dt))
  back = next((t[ia[k]] - dur for k in range(len(ia) - n) if ok[k:k + n].all()), 3.0)
  out["back_on_plan_s"] = float(back)
  # James (2026-09-28): the car after release, which only this sim can show (the model re-plans here). MetaDrive ground
  # truth, lane_gt.csv: lat_m from lane centre, heading_err in rad; to 6 s after release or the map end.
  gt = np.genfromtxt(f"{run}/frames/lane_gt.csv", delimiter=",", names=True, dtype=None, encoding=None)
  gt_t = gt["t_mono"] - starts[0]
  ga = (gt_t >= dur) & (gt_t < dur + 6)
  lat = np.asarray(gt["lat_m"], dtype=float)
  li = np.array([str(x) for x in gt["lane_idx"]])
  gp = (gt_t >= 0) & (gt_t < dur + 6)
  # lat_m is measured from the current lane's centre, so a lane change breaks the drift numbers: flag it
  out["lane_changed"] = bool(gp.any() and len(set(li[gp])) > 1)
  if ga.any():
    out["post_lat_peak_m"] = float(np.max(np.abs(lat[ga])))
    far = np.where(ga & (np.abs(lat) >= 0.3))[0]
    out["post_back_0p3m_s"] = float(gt_t[far[-1]] - dur) if len(far) else 0.0  # last time outside 0.3 m (6.0 = never back)
    out["post_hdg_peak_deg"] = float(np.degrees(np.max(np.abs(np.asarray(gt["heading_err"], dtype=float)[ga]))))
  out["plan_mean_hold"] = float(np.mean(plan[hold]))
  out["v_mean_hold"] = float(np.mean(z["v"][hold]))
  return out


if __name__ == "__main__":
  for r in sys.argv[1:]:
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in score(r).items()}))
