#!/usr/bin/env python3
"""D-071 evidence: NORMALIZED_CLOSING vs a future-range truth on railed Bosch-A rows (LOG/REPLAY only).

truth(t) = LSQ slope of the lead's ground position (dRel + ego odometry from vEgo) over the same track's measured
sweeps in [t+0.2, t+1.2] s, minus vEgo(t). It uses no NC, no U11 and no past range (RAIL_FAST uses past range),
so it is not circular. A lead's own acceleration inside the window biases it; it is a reference, not ground truth.
Usage: ncveto_truth.py <dir with pts_*.csv from ncveto_extract.py>
"""
import csv
import glob
import sys
from collections import defaultdict

import numpy as np

A, B = 0.2, 1.2
RAIL = -13.5


def load(sp):
  rows = []
  for f in sorted(glob.glob(f"{sp}/pts_*.csv")):
    route = f.split("pts_")[1][:8]
    with open(f) as fh:
      R = list(csv.DictReader(fh))
    ts = np.array([float(r["t"]) for r in R])
    vs = np.array([float(r["vego"]) for r in R])
    ut, ui = np.unique(ts, return_index=True)
    uv = vs[ui]
    xe = np.concatenate([[0.0], np.cumsum(0.5 * (uv[1:] + uv[:-1]) * np.diff(ut))])
    by = defaultdict(list)
    for r in R:
      if r["meas"] == "1":
        by[r["tid"]].append(r)
    for tid, L in by.items():
      T = np.array([float(r["t"]) for r in L])
      X = np.array([float(r["d"]) for r in L]) + np.interp(T, ut, xe)
      for i, r in enumerate(L):
        if float(r["v"]) > RAIL + 0.05 or r["nc_raw"] in ("", "512"):
          continue
        m = (T >= T[i] + A) & (T <= T[i] + B)
        if m.sum() < 8:
          continue
        truth = np.polyfit(T[m], X[m], 1)[0] - float(r["vego"])
        H = [-(int(q["nc_raw"]) - 512) / 64 * float(q["d"]) for q, tq in zip(L[max(0, i - 6):i + 1], T[max(0, i - 6):i + 1], strict=True)
             if q["nc_raw"] not in ("", "512") and T[i] - tq <= 0.5]
        rows.append({"route": route, "t": T[i], "tid": tid, "d": float(r["d"]), "y": float(r["y"]), "sig": int(r["nc_sig"]),
                     "nc": H[-1], "med": float(np.median(H[-5:])) if len(H) >= 3 else None, "truth": truth,
                     "vego": float(r["vego"])})
  return rows


def main():
  rows = load(sys.argv[1])
  moving = [r for r in rows if r["med"] is not None and r["vego"] + r["truth"] > -3.0]  # drop oncoming tracks
  print(f"railed measured rows with NC raw != 512 and a future truth: {len(rows)}; non-oncoming with a 5-sweep median: {len(moving)}")
  print("\n5-sweep NC median minus truth (+ = NC reads LESS closing), non-oncoming, sigma < 64")
  print("| dRel m | n | median | p10 | p90 | share > +3.5 |")
  print("|---|---|---|---|---|---|")
  for lo, hi in ((0, 50), (50, 75), (75, 100), (100, 150)):
    e = np.array([r["med"] - r["truth"] for r in moving if lo <= r["d"] < hi and r["sig"] < 64])
    if len(e):
      print(f"| {lo}-{hi} | {len(e)} | {np.median(e):+.2f} | {np.percentile(e, 10):+.2f} | {np.percentile(e, 90):+.2f} | {np.mean(e > 3.5):.3f} |")
  print("\nVeto rule on all rows: median >= rail + Y, dRel < 80, sigma < 64. LOST = truth <= rail - 1 (real closing past the rail)")
  print("| Y | fires (non-oncoming) | LOST (non-oncoming) | fires (all) | LOST (all) |")
  print("|---|---|---|---|---|")
  for Y in (2.0, 3.0, 3.5, 4.0, 5.0):
    fa = [r for r in rows if r["med"] is not None and r["d"] < 80 and r["sig"] < 64 and r["med"] >= RAIL + Y]
    fm = [r for r in fa if r["vego"] + r["truth"] > -3.0]
    print(f"| {Y} | {len(fm)} | {sum(r['truth'] <= -14.5 for r in fm)} | {len(fa)} | {sum(r['truth'] <= -14.5 for r in fa)} |")
  print("\nLOST non-oncoming rows at Y 3.5 (route, t, tid, dRel, sigma, NC, median, truth):")
  for r in moving:
    if r["d"] < 80 and r["sig"] < 64 and r["med"] >= RAIL + 3.5 and r["truth"] <= -14.5:
      print(f"  {r['route']} {int(r['t'] // 60)}:{r['t'] % 60:05.2f} {r['tid']} {r['d']:.1f} {r['sig']} {r['nc']:.1f} {r['med']:.1f} {r['truth']:.1f}")


if __name__ == "__main__":
  main()
