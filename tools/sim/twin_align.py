#!/usr/bin/env python3
"""Where the spread between twin runs comes from (sim expansion plan, determinism; the exit rows were voided on a 0.63 m
twin spread). MetaDrive runs with traffic off on a fixed map, so the world is the same every run; what differs is when
things happen (the model, the controls loop and the bridge are separate processes on wall-clock time). This splits a twin
set's lateral spread at release + x s into the parts a scorer can remove and the part it cannot:

  time      spread of lat_u at the same time since the trigger (what override_drift reports as twin_spread_rel*)
  distance  spread at the same distance along the road as the median twin at that time (trigger timing / speed removed)
  anchored  distance-aligned, minus each run's own lat at the trigger (the car's position when the press starts removed)
  shifted   time-aligned after each run's best time shift within +-0.5 s against the median (a pure delay removed)
and the inputs behind it per run: trigger time, distance and lat at the trigger, speed, the largest controls or sim-step
stall (> 100 ms counts) from 0.5 s before the trigger to release + 3 s.

  tools/sim/twin_align.py --press SCEN [--dur S] RUN [RUN ...]
    SCEN names the press whose trigger rule sets t = 0 for every run (e.g. hug_45 for the handsoff_45 twins); --dur
    overrides its duration (release = t = dur). Runs are recorder directories (frames/lane_gt.csv, lat_pid_sim.npz,
    lanes.npz).

If "distance" or "anchored" is well under "time", the twins differ in timing, not in path, and exit rows can be scored
distance-aligned from the trigger. If none of them shrink it, the runs drove different paths: rerun, and check the stall
column first. Sim evidence only.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from openpilot.tools.sim.override_drift import stall, trigger_t  # noqa: E402
from openpilot.tools.sim.override_score import SCEN  # noqa: E402


def load(run, press):
  gt = np.genfromtxt(f"{run}/frames/lane_gt.csv", delimiter=",", names=True, dtype=None, encoding=None, converters={"lane_idx": str})
  gt_t, lat, v = gt["t_mono"], np.asarray(gt["lat_m"], dtype=float), np.asarray(gt["speed"], dtype=float)
  z = np.load(f"{run}/lat_pid_sim.npz")
  tm = np.load(f"{run}/lanes.npz")["t_mono"]
  t0 = trigger_t(tm, z["des_angle"], press, z["v"])
  if t0 is None:
    return None
  d = np.diff(lat)
  lat_u = lat - np.concatenate(([0.0], np.cumsum(np.where(np.abs(d) > 2.0, d, 0.0))))
  s = np.concatenate(([0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(gt_t))))
  return dict(run=os.path.basename(run.rstrip("/")), t=gt_t - t0, s=s, lat=lat_u, v=v, frame=np.asarray(gt["frame"], dtype=float),
              tm=tm - t0, t0=float(t0))


def best_shift(t, lat, t_ref, lat_ref, x, max_s=0.5):
  """The time shift (s) within +-max_s that best matches lat to lat_ref over [x - 1, x + 1] s."""
  g = np.arange(x - 1.0, x + 1.0, 0.01)
  ref = np.interp(g, t_ref, lat_ref)
  shifts = np.arange(-max_s, max_s + 1e-9, 0.01)
  err = [np.mean((np.interp(g + sh, t, lat) - ref) ** 2) for sh in shifts]
  return float(shifts[int(np.argmin(err))])


def align(runs, dur, xs=(1.0, 2.0, 3.0)):
  out = {"runs": [{"run": r["run"], "s_trigger_m": round(float(np.interp(0.0, r["t"], r["s"])), 2),
                   "lat_trigger_m": round(float(np.interp(0.0, r["t"], r["lat"])), 3),
                   "v_trigger": round(float(np.interp(0.0, r["t"], r["v"])), 2),
                   "stall_ms": round(1e3 * stall(r, dur + 3.0), 1)} for r in runs]}
  med_t = np.arange(-1.0, dur + 4.0, 0.01)
  med_lat = np.median([np.interp(med_t, r["t"], r["lat"]) for r in runs], axis=0)
  for x in xs:
    tx = dur + x
    lt = [np.interp(tx, r["t"], r["lat"]) for r in runs]
    s_med = float(np.median([np.interp(tx, r["t"], r["s"]) - np.interp(0.0, r["t"], r["s"]) for r in runs]))
    ld = [np.interp(np.interp(0.0, r["t"], r["s"]) + s_med, r["s"], r["lat"]) for r in runs]
    la = [a - np.interp(0.0, r["t"], r["lat"]) for a, r in zip(ld, runs, strict=True)]
    sh = [best_shift(r["t"], r["lat"], med_t, med_lat, tx) for r in runs]
    ls = [np.interp(tx + k, r["t"], r["lat"]) for k, r in zip(sh, runs, strict=True)]
    out[f"rel{x:g}s"] = {"time_m": round(float(np.ptp(lt)), 3), "distance_m": round(float(np.ptp(ld)), 3),
                         "anchored_m": round(float(np.ptp(la)), 3), "shifted_m": round(float(np.ptp(ls)), 3),
                         "shifts_s": [round(k, 2) for k in sh]}
  return out


def main(argv):
  ap = argparse.ArgumentParser()
  ap.add_argument("--press", required=True)
  ap.add_argument("--dur", type=float)
  ap.add_argument("runs", nargs="+")
  a = ap.parse_args(argv)
  press = SCEN[a.press]["press"] or SCEN[a.press].get("virtual_press")
  dur = a.dur if a.dur is not None else press["dur"]
  runs = [r for r in (load(x, press) for x in a.runs) if r is not None]
  if len(runs) < 2:
    sys.exit("need two or more runs that reach the trigger")
  print(json.dumps(align(runs, dur)))


if __name__ == "__main__":
  main(sys.argv[1:])
