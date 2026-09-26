#!/usr/bin/env python3
"""Per-route lateral checks for an owner report: low-speed torque chatter, and turn-in vs unwind tracking.

Built for STATUS 143 (routes 00000278 and 0000027a: low-speed stutter with NrdrLatRateFF 0.5, slow unwind).
Reads the lat_pid_sim.py per-frame cache (extracted on first use, next to the route, never in the repo).

  chatter   engaged, hands-off frames by speed band: rms frame-to-frame change of the PID output (% of full
            torque), torque direction reversals per second (|step| > 0.3 %), and wheel jitter (angle minus its
            0.25 s centred mean, rms deg).
  unwind    engaged low-speed (< 25 mph) turns with a peak |desired| >= 45 deg and < 5 % pressed frames. Each is
            split at the peak into turn-in and unwind; per phase the best time shift of angle onto desired
            (lag) and the mean error in the phase's direction (> 0 = wheel behind the request). I@peak is the
            logged integrator at the peak, signed into the turn (> 0 = it opposes the unwind).
  rateff    open-loop replay (logged freeze) with NrdrLatRateFF 0.5 and 0 in 30 s windows; whichever matches the
            logged output tells when the param was live (initData only records the value at the route start).

Usage:
  python tools/lateral/lat_route_check.py chatter ROUTE_DIR[:T0-T1] [...]
  python tools/lateral/lat_route_check.py unwind ROUTE_DIR [...]
  python tools/lateral/lat_route_check.py rateff ROUTE_DIR

Log-decode and open-loop replay evidence only. Needs PARAMS_ROOT set (rateff builds the real controller).
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lat_pid_sim as sim

MPH = sim.MPH
REV_STEP = 0.003


def _route(spec):
  path, _, rng = spec.partition(":")
  t0, t1 = (float(x) for x in rng.split("-")) if rng else (0.0, 1e9)
  return sim.extract(path), t0, t1, rng or "all"


def chatter(d, m):
  out, ang = d["out"], d["angle"]
  du = np.r_[0.0, np.diff(out)]
  hp = ang - np.convolve(ang, np.ones(25) / 25, mode="same")
  sg = np.sign(np.where(np.abs(du) > REV_STEP, du, 0.0))[m]
  sg = sg[sg != 0]
  rev = int(np.sum(sg[1:] != sg[:-1])) if len(sg) > 1 else 0
  sec = m.sum() / 100
  return sec, float(np.sqrt(np.mean(du[m] ** 2)) * 100), rev / max(sec, 1e-9), float(np.sqrt(np.mean(hp[m] ** 2)))


def _lag(des, ang):
  best = (np.inf, 0)
  for sh in range(0, min(61, len(des) // 2), 2):
    e = np.mean((ang[sh:] - des[:len(des) - sh]) ** 2)
    best = min(best, (e, sh))
  return best[1] / 100


def unwind_episodes(d, peak_min=45.0, vmax_mph=25.0):
  t, des, ang, v = d["t"], d["des_angle"], d["angle"], d["v"]
  big = (d["active"] > 0.5) & (v > 2) & (v < vmax_mph * MPH) & (np.abs(des) > 15)
  eps, i, n = [], 0, len(t)
  while i < n:
    if not big[i]:
      i += 1
      continue
    j = i
    while j < n and big[j] and np.sign(des[j]) == np.sign(des[i]):
      j += 1
    seg = slice(i, j)
    a = np.abs(des[seg])
    pk = i + int(np.argmax(a))
    if (j - i > 100 and a.max() >= peak_min and (d["pressed"][seg] > 0.5).mean() < 0.05
        and not (d["lane_change"][seg] > 0).any() and pk - i > 40 and j - pk > 40):
      sg = np.sign(des[i])
      tin, tout = slice(i, pk), slice(pk, j)
      eps.append(dict(t=t[pk], peak=a.max(), v=v[pk] / MPH,
                      lag_in=_lag(des[tin], ang[tin]), lag_out=_lag(des[tout], ang[tout]),
                      err_in=float(np.mean(sg * (des[tin] - ang[tin]))), err_out=float(np.mean(sg * (ang[tout] - des[tout]))),
                      i_pk=float(sg * d["log_i"][pk])))
    i = j
  return eps


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("cmd", choices=("chatter", "unwind", "rateff"))
  ap.add_argument("routes", nargs="+")
  a = ap.parse_args(argv)

  for spec in a.routes:
    d, t0, t1, rng = _route(spec)
    name = d["route"]
    win = (d["t"] >= t0) & (d["t"] < t1) & (d["active"] > 0.5)
    if a.cmd == "chatter":
      for lo, hi in ((2, 10), (10, 25), (25, 50), (50, 90)):
        m = win & (d["pressed"] < 0.5) & (d["v"] >= lo * MPH) & (d["v"] < hi * MPH)
        if m.sum() < 500:
          continue
        sec, du, rev, hp = chatter(d, m)
        print(f"{name} {rng:>10} {lo:2d}-{hi:2d} mph {sec:6.0f} s  d(torque) rms {du:.2f} %/frame  " +
              f"reversals {rev:5.1f} /s  wheel jitter {hp:.2f} deg")
    elif a.cmd == "unwind":
      E = unwind_episodes(d)
      if not E:
        print(f"{name} no episodes")
        continue
      med = {k: float(np.median([e[k] for e in E])) for k in E[0]}
      print(f"{name} n={len(E):3d}  lag in/out {med['lag_in']:.2f}/{med['lag_out']:.2f} s  " +
            f"err in/out {med['err_in']:5.1f}/{med['err_out']:5.1f} deg  I@peak {med['i_pk']:+.3f}")
    else:
      on = sim.replay_logged_freeze(d, {"NrdrLatRateFF": "0.5"})
      off = sim.replay_logged_freeze(d, {"NrdrLatRateFF": "0.0"})
      for w0 in np.arange(0, d["t"][-1], 30):
        m = win & (d["t"] >= w0) & (d["t"] < w0 + 30)
        if m.sum() < 300:
          continue
        e1, e0 = np.median(np.abs(on[m] - d["out"][m])), np.median(np.abs(off[m] - d["out"][m]))
        print(f"{name} {int(w0 // 60):3d}:{int(w0 % 60):02d}  v {np.median(d['v'][m]) / MPH:3.0f} mph  " +
              f"|sim-log| rff 0.5 {e1:.1e}  rff 0 {e0:.1e}  -> {'0.5' if e1 < e0 else '0'}")


if __name__ == "__main__":
  main()
