#!/usr/bin/env python3
"""PR 10 sim metrics (prereg /tmp/simroutes/pr10_prereg.txt 099af3c8, signed John/Kevin/James 2026-09-28). Sim evidence only.

  tools/sim/pr10_metrics.py RUN [RUN ...]    # one JSON line per run

Per run, frames at the controls rate (lat_pid_sim.npz, ~100 Hz) with latActive, no driver press and |eps_torque| < 400 are
split into speed bands 5-8, 8-12, 12-16, 16-25, 25+ m/s. Per band:
  secs       hands-off seconds in the band (a band counts in a row only with >= 60 s pooled over its seeds)
  rms_err    RMS(des_angle - angle), deg
  turn_err   the same over |des_angle| > 20 deg (None when no such frame)
  wig        RMS of the 0.6-2 Hz band of the wheel angle / RMS of the same band of des_angle
  rough      RMS of the 5-8 Hz band of cc_torque / RMS(cc_torque)
  step_p99 / step_max   |per-frame change| of cc_torque
Band-passes are zero-phase FFT masks over the whole run (uniform resample at the median dt, mean removed), then the
frames of each band are selected, so a band's edges do not truncate the filter.
Per run (lane_gt.csv lat_m, + = right, interpolated to the controls clock):
  lane_off_med  median lat over the hands-off frames (straights, F6)
  inside_off    mean inside offset over curve frames (|des_angle| >= half its max, one sign); + = toward the curve's inside
                (wheel + = left, so inside = -lat * sign(des_angle)) (R3, F3)
  stall_ms      largest controls or sim-step gap > 100 ms inside the scored span (first to last hands-off frame at
                >= 5 m/s), else 0; start-up gaps before the car is moving and engaged do not count
Gap mask (prereg Amendment 1): every controls gap or sim-step gap > 100 ms inside the scored span is masked. Frames within
+-0.5 s of a gap are removed from rms_err, turn_err, offsets and step_p99/step_max. Frames within +-1.5 s are removed from
wig and rough. Band-passes run on the full series and are masked afterwards. A step is also dropped if it differs across a
gap, i.e. if either of its two frames is masked. mask05_s / mask15_s = masked seconds of the scored span; mask05_pct /
mask15_pct as %. The run is void when mask05_pct > 10 or mask15_pct > 15. Both arms use the same code.
Also per band: wheel_bp = RMS of the 0.6-2 Hz band of the wheel angle (deg), report only (Kevin; R3 curve wiggle, no ratio);
des_bp = RMS of the 0.6-2 Hz band of des_angle (deg), so a wig ratio over a near-empty band is visible.
"""
import json
import sys

import numpy as np

BANDS = [("5-8", 5.0, 8.0), ("8-12", 8.0, 12.0), ("12-16", 12.0, 16.0), ("16-25", 16.0, 25.0), ("25+", 25.0, 99.0)]


def bandpass(x, dt, lo, hi):
  x = np.asarray(x, float) - np.mean(x)
  f = np.fft.rfftfreq(len(x), dt)
  X = np.fft.rfft(x)
  X[(f < lo) | (f > hi)] = 0.0
  return np.fft.irfft(X, len(x))


def rms(x):
  return float(np.sqrt(np.mean(np.square(x)))) if len(x) else None


def run_metrics(run):
  z = np.load(f"{run}/lat_pid_sim.npz")
  t = np.asarray(z["t"], float)
  dt = float(np.median(np.diff(t)))
  tu = np.arange(t[0], t[-1], dt)
  u = {k: np.interp(tu, t, np.asarray(z[k], float)) for k in ("angle", "des_angle", "cc_torque", "v", "eps_torque")}
  u["active"] = np.interp(tu, t, np.asarray(z["active"], float)) > 0.5
  u["pressed"] = np.interp(tu, t, np.asarray(z["pressed"], float)) > 0.5
  ok = u["active"] & ~u["pressed"] & (np.abs(u["eps_torque"]) < 400)
  w_bp, d_bp = bandpass(u["angle"], dt, 0.6, 2.0), bandpass(u["des_angle"], dt, 0.6, 2.0)
  cc = u["cc_torque"]
  cc_bp = bandpass(cc, dt, 5.0, 8.0)
  step = np.abs(np.diff(cc, prepend=cc[0]))
  err = u["des_angle"] - u["angle"]
  gt = np.genfromtxt(f"{run}/frames/lane_gt.csv", delimiter=",", names=True, dtype=None, encoding=None, converters={"lane_idx": str})
  tm = np.load(f"{run}/lanes.npz")["t_mono"]  # sample-aligned with t
  sc = ok & (u["v"] >= 5.0)
  t0, t1 = (tu[sc][0], tu[sc][-1]) if sc.any() else (t[-1], t[-1])
  gaps = [(a, b) for a, b in zip(t[:-1], t[1:]) if b - a > 0.1 and b > t0 and a < t1]
  gt_t = np.asarray(gt["t_mono"], float)
  sd = np.diff(gt_t) / np.maximum(np.diff(np.asarray(gt["frame"], float)), 1)
  for k in np.where(sd > 0.1)[0]:
    a, b = np.interp([gt_t[k], gt_t[k + 1]], tm, t)
    if b > t0 and a < t1:
      gaps.append((a, b))
  g = float(max([b - a for a, b in gaps] + [0.0]))  # longest gap on the controls clock
  m05, m15 = np.zeros(len(tu), bool), np.zeros(len(tu), bool)
  for a, b in gaps:
    m05 |= (tu >= a - 0.5) & (tu <= b + 0.5)
    m15 |= (tu >= a - 1.5) & (tu <= b + 1.5)
  span = sc.sum()
  step_ok = ~m05 & ~np.roll(m05, 1)
  ok05, ok15 = ok & ~m05, ok & ~m15
  out = {"run": run.rstrip("/").split("/")[-1], "bands": {}}
  for name, lo, hi in BANDS:
    vb = (u["v"] >= lo) & (u["v"] < hi)
    m, mb = ok05 & vb, ok15 & vb
    if not m.any():
      continue
    tn = m & (np.abs(u["des_angle"]) > 20.0)
    ms = m & step_ok
    dr = rms(d_bp[mb]) if mb.any() else None
    out["bands"][name] = {"secs": round(float(m.sum() * dt), 1), "secs15": round(float(mb.sum() * dt), 1), "rms_err": round(rms(err[m]), 3),
                          "turn_err": round(rms(err[tn]), 3) if tn.any() else None,
                          "wig": round(rms(w_bp[mb]) / dr, 3) if dr else None, "des_bp": round(dr, 4) if dr else None, "wheel_bp": round(rms(w_bp[mb]), 4) if mb.any() else None,
                          "rough": round(rms(cc_bp[mb]) / rms(cc[mb]), 3) if mb.any() and rms(cc[mb]) else None,
                          "step_p99": round(float(np.percentile(step[ms], 99)), 4) if ms.any() else None,
                          "step_max": round(float(step[ms].max()), 4) if ms.any() else None}
  lat = np.interp(np.interp(tu, t, tm), gt_t, np.asarray(gt["lat_m"], float))
  out["lane_off_med"] = round(float(np.median(lat[ok05])), 3) if ok05.any() else None
  des = u["des_angle"]
  sgn = np.sign(des[np.argmax(np.abs(des))])
  cm = ok05 & (sgn * des >= 0.5 * np.abs(des).max())
  out["inside_off"] = round(float(np.mean(-lat[cm] * sgn)), 3) if cm.any() and np.abs(des).max() > 5.0 else None
  out["mask05_s"], out["mask15_s"] = round(float((sc & m05).sum() * dt), 1), round(float((sc & m15).sum() * dt), 1)
  out["mask05_pct"] = round(100.0 * (sc & m05).sum() / span, 1) if span else None
  out["mask15_pct"] = round(100.0 * (sc & m15).sum() / span, 1) if span else None
  out["void"] = bool(span == 0 or out["mask05_pct"] > 10.0 or out["mask15_pct"] > 15.0)
  out["n_gaps"] = len(gaps)
  out["stall_ms"] = round(1e3 * g, 1) if g > 0.1 else 0.0
  return out


if __name__ == "__main__":
  for r in sys.argv[1:]:
    print(json.dumps(run_metrics(r)))
