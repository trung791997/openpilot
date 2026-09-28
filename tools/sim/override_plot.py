#!/usr/bin/env python3
"""One picture per MetaDrive override episode, for the override peers (hugging, over- and understeer are easier to judge
seen than scored): wheel angle vs plan vs the driver's want, driver and delivered torque, the car's lateral offset in
its lane (MetaDrive ground truth, lane_gt.csv), steeringPressed, and three road-camera frames (press start, end of the
hold, 1 s after release).

  tools/sim/override_plot.py RUN_DIR [OUT.png]     (default RUN_DIR/override.png; t = 0 at the press start)
Sim evidence only.
"""
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from openpilot.tools.sim.override_score import SCEN, press_start, want_trace  # noqa: E402


def main(run, out=None):
  name = os.path.basename(run.rstrip("/"))
  z = np.load(f"{run}/lat_pid_sim.npz")
  tm = np.load(f"{run}/lanes.npz")["t_mono"]
  scen, press, t0 = press_start(run)
  t = tm - (t0 if t0 is not None else tm[0])
  ang, plan = z["angle"], z["des_angle"]
  want = want_trace(press, t, plan)
  gt = list(csv.DictReader(open(f"{run}/frames/lane_gt.csv")))
  gt_t = np.array([float(r["t_mono"]) for r in gt]) - (t0 if t0 is not None else tm[0])
  gt_lat = np.array([float(r["lat_m"]) if r["lane_idx"] not in ("None", "") else np.nan for r in gt])
  gt_fr = np.array([int(r["frame"]) for r in gt])
  dur = press["dur"] if press else 0.0
  sel = (t > -4) & (t < dur + 6)
  fig = plt.figure(figsize=(13, 10))
  gs = fig.add_gridspec(5, 3, height_ratios=[2.2, 1.4, 1.2, 0.5, 2.0])
  a1 = fig.add_subplot(gs[0, :])
  a1.plot(t[sel], plan[sel], "k--", label="plan (controller's desired angle)")
  if want is not None:
    a1.plot(t[sel], want[sel], "g:", lw=2, label="driver wants")
  a1.plot(t[sel], ang[sel], "b", label="wheel angle")
  a1.set_ylabel("deg"); a1.legend(loc="best", fontsize=8)
  a1.set_title(f"{name}  (scenario {scen}; v {np.mean(z['v'][sel]):.1f} m/s)  sim, car as driven")
  a2 = fig.add_subplot(gs[1, :], sharex=a1)
  a2.plot(t[sel], z["eps_torque"][sel], "g", label="driver torque (STEER_TORQUE_SENSOR)")
  a2b = a2.twinx(); a2b.plot(t[sel], z["co_torque"][sel], "r", label="delivered (carOutput, -1..1)"); a2b.set_ylim(-1.05, 1.05)
  a2.set_ylabel("sensor units"); a2.legend(loc="upper left", fontsize=8); a2b.legend(loc="upper right", fontsize=8)
  a3 = fig.add_subplot(gs[2, :], sharex=a1)
  s3 = (gt_t > -4) & (gt_t < dur + 6)
  a3.plot(gt_t[s3], gt_lat[s3], "m"); a3.axhline(0, color="k", lw=0.5); a3.set_ylabel("lane offset m\n(step = lane change)")
  a4 = fig.add_subplot(gs[3, :], sharex=a1)
  a4.fill_between(t[sel], 0, z["pressed"][sel], step="post", color="orange"); a4.set_yticks([]); a4.set_ylabel("pressed")
  a4.set_xlabel("s from press start")
  for ax in (a1, a2, a3, a4):
    ax.axvspan(0, dur, color="yellow", alpha=0.15)
  jpgs = sorted(glob.glob(f"{run}/frames/*.jpg"))
  jfr = np.array([int(os.path.basename(j)[:-4]) for j in jpgs])
  for k, (tt, lab) in enumerate(((0.0, "press start"), (dur - 0.2, "end of hold"), (dur + 1.0, "release + 1 s"))):
    ax = fig.add_subplot(gs[4, k]); ax.axis("off")
    if len(jfr) and np.isfinite(tt):
      fr = gt_fr[np.argmin(np.abs(gt_t - tt))]
      ax.imshow(plt.imread(jpgs[int(np.argmin(np.abs(jfr - fr)))])[:, :, ::-1]); ax.set_title(f"{lab} (t {tt:.1f} s)", fontsize=9)
  fig.tight_layout()
  out = out or f"{run}/override.png"
  fig.savefig(out, dpi=80)
  print(out)


if __name__ == "__main__":
  main(*sys.argv[1:3])
