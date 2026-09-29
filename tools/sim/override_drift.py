#!/usr/bin/env python3
"""Drift after release, measured against the hands-off twins (James's rule via Kevin, 2026-09-28). The model's own plan runs
wide on the 45 mph override maps (hands-off drifts ~1 m outward in 10 s), so the raw post-release lateral peak mostly
measures the plan. Each run's lateral position (MetaDrive lane_gt.csv lat_m, + = right, unwrapped across lane changes so
it is road-referenced) is compared with the median of its hands-off twins at the same distance along the road (distance
from the spawn, integrated speed; every run on a map spawns at the same point and the press trigger is the same rule).

  tools/sim/override_drift.py --twins TWIN [TWIN ...] -- RUN [RUN ...]
    (grab_45_* and nudge_45: handsoff_n45 twins; hug_45 and poshug_45_*: handsoff_45 twins)

excess = lat_u - twin median, signed + toward the driver's push (the lateral side the hand steers to during the press).
Window: release to the earliest of release + 3 s, the run's first lane-line crossing (lane_idx change) or the twin
median's first crossing (|median| >= lane width / 2); window_end says which (3s / run / twin).
  push_excess_peak_m = max(+excess), far_excess_peak_m = max(-excess) (past the twin on the far side; flag > 0.1 m)
rel_push_m = max displacement toward the push from the run's own lat at release, over the window (ret_peak_m: the same away from the push = how far it came back; NOT a far-side measure); vlat_rel = lateral
speed toward the push at release (m/s, line fit over release +-0.1 s).
rel_push_3s_m is the DECIDING long-hold columns. BUG FIX (2026-09-28 ~09:04 CDT, James via Kevin), not a
rule change: the approved rule was always release -> +3 s; the tool wrongly cut it at lane crossings. Fixed
window release -> +3 s, not cut at lane crossings, because lat is unwrapped and cutting at a crossing made A's hug_30
rel_push 0.037 m (crossed 0.09 s after release) against C's full 3 s. Fixed before any n=5 long-hold result existed. rel_push_6s_m: +6 s, information only.
rel_lat_10s_m = push-side travel from the release position at t = 10 s. vlat_rel already uses release +-0.1 s.
release_step_max = largest per-frame |change| in delivered torque (carOutput, -1..1) over [release, release + 0.5 s).
Twin stability: spread (max - min) of the twins' lat_u at release + 1, 2, 3 s (by the run's press duration and the twins'
own trigger). Sim evidence only.
"""
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from openpilot.tools.sim.override_score import SCEN, want_trace  # noqa: E402

LANE_W = 3.5  # MetaDrive default; the measured jump at a lane change is used when there is one


def trigger_t(tm, plan, press, v=None):
  """The driver model's trigger rule: |plan| >= trigger_deg, or v_ego >= trigger_v (straights), held for dwell s."""
  above = (v >= press["trigger_v"]) if "trigger_v" in press else (np.abs(plan) >= press["trigger_deg"])
  dt, run_s = float(np.median(np.diff(tm))), 0.0
  for k in range(len(tm)):
    run_s = run_s + dt if above[k] else 0.0
    if run_s >= press.get("dwell", 1.0):
      return tm[k]
  return None


def trace(run):
  name = os.path.basename(run.rstrip("/"))
  scen = re.match(r"ovr_(.+)_\d+_\d+_[^_]+$", name)[1]
  gt = np.genfromtxt(f"{run}/frames/lane_gt.csv", delimiter=",", names=True, dtype=None, encoding=None, converters={"lane_idx": str})  # lane_idx is "None" past the map end
  gt_t, lat, v = gt["t_mono"], np.asarray(gt["lat_m"], dtype=float), np.asarray(gt["speed"], dtype=float)
  li = np.array([str(x) for x in gt["lane_idx"]])
  z = np.load(f"{run}/lat_pid_sim.npz")
  tm = np.load(f"{run}/lanes.npz")["t_mono"]
  press = SCEN[scen]["press"] or SCEN[scen].get("virtual_press")
  starts = re.findall(r"sim_driver: press \d+ start ([\d.]+)", open(f"{run}/bridge.log", errors="ignore").read())
  t0 = float(starts[0]) if (starts and SCEN[scen]["press"]) else trigger_t(tm, z["des_angle"], press, z["v"])
  d = np.diff(lat)
  jump = np.abs(d) > 2.0
  lat_u = lat - np.concatenate(([0.0], np.cumsum(np.where(jump, d, 0.0))))
  s = np.concatenate(([0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(gt_t))))
  lane_w = float(np.median(np.abs(d[jump]))) if jump.any() else LANE_W
  return dict(scen=scen, press=press, frame=np.asarray(gt["frame"], dtype=float), t=gt_t - t0, s=s, lat=lat_u, li=li, lane_w=lane_w, tm=tm - t0, z=z)


def stall(r, t_end):
  """Largest controls or sim-step gap (s) over (press - 0.5 s, t_end) when it is > 100 ms, else 0.0 (Kevin/James)."""
  ct, st = r["tm"], r["t"]
  cg = np.diff(ct)[(ct[1:] >= -0.5) & (ct[:-1] < t_end) & (np.diff(ct) > 0.1)]
  sd = np.diff(st) / np.maximum(np.diff(r["frame"]), 1)
  sg = sd[(st[1:] >= -0.5) & (st[:-1] < t_end) & (sd > 0.1)]
  return float(max(list(cg) + list(sg) + [0.0]))


def push_side(r):
  """+1 when the driver's push moves the car right (+lat), -1 left. Wheel angle + = left, so lateral side = -sign(dwheel)."""
  p, t, z = r["press"], r["tm"], r["z"]
  hold = (t >= 0) & (t < p["dur"])
  if p["kind"] == "position":
    dw = np.mean((z["angle"] - z["des_angle"])[hold])
  else:
    dw = np.mean((want_trace(p, t, z["des_angle"]) - z["des_angle"])[hold])
  return -float(np.sign(dw)) or 1.0


def main(argv):
  i = argv.index("--")
  twins = [trace(r) for r in argv[argv.index("--twins") + 1:i]]
  print(json.dumps({"twin_stall_ms": {os.path.basename(a.rstrip("/")): round(1e3 * stall(w, w["press"]["dur"] + 3.0), 1)
                                      for a, w in zip(argv[argv.index("--twins") + 1:i], twins)}}))
  for run in argv[i + 1:]:
    r = trace(run)
    dur, t = r["press"]["dur"], r["t"]
    base = np.median([np.interp(r["s"], w["s"], w["lat"]) for w in twins], axis=0)
    ps = push_side(r)
    ex = (r["lat"] - base) * ps
    ends = {"3s": dur + 3.0}
    rel = np.where((t >= dur) & (r["li"] != r["li"][np.argmin(np.abs(t - dur))]))[0]
    if len(rel):
      ends["run"] = float(t[rel[0]])
    tw_x = np.where((t >= dur) & (np.abs(base) >= r["lane_w"] / 2))[0]
    if len(tw_x):
      ends["twin"] = float(t[tw_x[0]])
    end_k = min(ends, key=ends.get)
    ga = (t >= dur) & (t < ends[end_k])
    out = {"run": os.path.basename(run.rstrip("/")), "twins": len(twins), "push_side": "right" if ps > 0 else "left",
           "window_end": end_k, "window_s": ends[end_k] - dur,
           "push_excess_peak_m": float(np.max(ex[ga])) if ga.any() else None,
           "far_excess_peak_m": float(np.max(-ex[ga])) if ga.any() else None,
           "excess_at_release_m": float(np.interp(dur, t, ex)),
           "lane_changed": bool(len(set(r["li"][(t >= 0) & (t < dur + 6)])) > 1),
           "crossed_before_release": bool(len(set(r["li"][(t >= 0) & (t <= dur)])) > 1)}  # then the window starts in the new lane
    # Kevin: a controls or sim stall > 100 ms from press - 0.5 s to the window end decides a run by itself: flag and rerun
    ct, st = r["tm"], t
    cg = np.where((ct[1:] >= -0.5) & (ct[:-1] < ends[end_k]) & (np.diff(ct) > 0.1))[0]
    sg = np.where((st[1:] >= -0.5) & (st[:-1] < ends[end_k]) & (np.diff(st) / np.maximum(np.diff(r["frame"]), 1) > 0.1))[0]
    out["stall_in_window"] = bool(len(cg) or len(sg))
    out["max_gap_ms"] = float(1e3 * max([np.diff(ct)[k] for k in cg] + [np.diff(st)[k] for k in sg] + [0.0]))
    # Kevin/James: displacement toward the push from the car's own lateral position at release, and lateral speed at release
    rp = (r["lat"] - np.interp(dur, t, r["lat"])) * ps
    out["rel_push_m"] = float(np.max(rp[ga])) if ga.any() else None
    out["ret_peak_m"] = float(np.max(-rp[ga])) if ga.any() else None  # return from the release position (renamed from rel_far_m: not a far-side measure)
    g3 = (t >= dur) & (t < dur + 3.0)  # fixed window, not cut at lane crossings (lat is unwrapped): A/C comparable
    out["rel_push_3s_m"], out["ret_peak_3s_m"] = float(np.max(rp[g3])), float(np.max(-rp[g3]))
    # James: return toward/past centre after release (position rows): distance back by +3 s, peak return speed (0.2 s fit)
    # James: how far the hand got during the hold, and what the driver sees at release + 3 s (both from lat at press)
    hd = (t >= 0) & (t <= dur)
    lp = float(np.interp(0.0, t, r["lat"]))
    out["achieved_disp_m"] = float(np.max((r["lat"][hd] - lp) * ps)) if hd.any() else None
    out["resid_3s_m"] = float((np.interp(dur + 3.0, t, r["lat"]) - lp) * ps)
    out["lat_press_m"], out["lat_release_m"] = lp, float(np.interp(dur, t, r["lat"]))
    out["ret_3s_m"] =float(-np.interp(dur + 3.0, t, rp))
    k = np.where(g3)[0]
    vs = [np.polyfit(t[k[j:j + 5]], rp[k[j:j + 5]], 1)[0] for j in range(0, max(len(k) - 5, 0))]
    out["ret_vmax"] = float(max([-x for x in vs] + [0.0]))
    g6 = (t >= dur) & (t < dur + 6.0)  # information only (James), not a pass rule
    out["rel_push_6s_m"], out["ret_peak_6s_m"] = float(np.max(rp[g6])), float(np.max(-rp[g6]))
    out["rel_lat_10s_m"] = float(np.interp(10.0, t, rp)) if t[-1] >= 10.0 else None
    vw = (t >= dur - 0.1) & (t <= dur + 0.1)
    out["vlat_rel"] = float(np.polyfit(t[vw], r["lat"][vw], 1)[0] * ps) if vw.sum() >= 3 else None
    # Kevin/James: re-engage step = largest per-frame change in delivered torque (carOutput, -1..1) over [release, +0.5 s)
    rs = (ct[1:] >= dur) & (ct[1:] < dur + 0.5)
    out["release_step_max"] = float(np.abs(np.diff(r["z"]["co_torque"]))[rs].max()) if rs.any() else None
    # fx on a FIXED release -> +3 s window on unwrapped lat and unwrapped twin median (Kevin/James 2026-09-28 ~09:45 CDT): the
    # old window stopped at a lane crossing, the very event far_flag exists to catch. far_excess_peak_m (old window) kept.
    out["far_excess_3s_m"], out["push_excess_3s_m"] = float(np.max(-ex[g3])), float(np.max(ex[g3]))
    out["far_flag_value_m"] = out["far_excess_3s_m"]  # the only far-side number (twin-referenced)
    out["far_flag"] = out["far_flag_value_m"] > 0.1
    for x in (1, 2, 3):
      out[f"twin_spread_rel{x}s_m"] = float(np.ptp([np.interp(dur + x, w["t"], w["lat"]) for w in twins]))
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in out.items()}))


if __name__ == "__main__":
  main(sys.argv[1:])
