#!/usr/bin/env python3
"""Lateral tracking metrics of the PID lateral controller, from rlogs of the car or from a sim telemetry log.

  tools/sim/lat_metrics.py RLOG_OR_JSONL [...]   # one metrics block per file, plus a pooled one

Reads controlsState.lateralControlState.pidState (angleError, steeringAngleDesiredDeg, steeringAngleDeg,
output, saturated) with carState.vEgo and carControl.latActive, and scores samples while openpilot steered
with no driver torque, by speed band around the tune's 25 mph handoff:
  rms_err / p90_err   angle error, deg
  lag_s               delay of the measured angle behind the desired angle (cross-correlation, +-1 s)
  osc_hz              sign changes per second of the angle error while |error| > 0.5 deg (hunting)
  rate_rms            steering rate RMS, deg/s (activity)
  sat_frac            fraction of samples with the output saturated
  mean_abs_out        mean |output torque| (0..1)
Inputs: an rlog, a lat_pid_sim route cache (.npz, e.g. from tools/sim/sim_lat_record.py), or the sim telemetry jsonl
(records with "pid": {...} at 20 Hz).
"""
import json
import sys

import numpy as np

BANDS = {"lt25mph": (0.0, 11.176), "ge25mph": (11.176, 99.0)}
RATE = 20.0  # Hz the samples are resampled to


def from_rlog(path):
  from openpilot.tools.lib.logreader import LogReader
  v, lat_active, pressed = 0.0, False, False
  rows = []
  for m in LogReader(path):
    w = m.which()
    if w == "carState":
      v, pressed = m.carState.vEgo, m.carState.steeringPressed
    elif w == "carControl":
      lat_active = m.carControl.latActive
    elif w == "controlsState" and m.controlsState.lateralControlState.which() == "pidState":
      p = m.controlsState.lateralControlState.pidState
      rows.append((m.logMonoTime * 1e-9, v, p.steeringAngleDesiredDeg, p.steeringAngleDeg, p.angleError, p.output, p.saturated,
                   lat_active and not pressed))
  return np.array(rows, dtype=np.float64)


def from_jsonl(path):
  rows = []
  for line in open(path):
    r = json.loads(line)
    if r.get("pid") and r.get("ok") is not False:
      p = r["pid"]
      rows.append((r["t"], r["v"], p["des"], p["meas"], p["err"], p["out"], p["sat"], r["active"]))
  return np.array(rows, dtype=np.float64)


def from_npz(path):
  """A lat_pid_sim route cache (tools/sim/sim_lat_record.py output or lat_pid_sim.extract cache); no saturation flag."""
  z = np.load(path, allow_pickle=False)
  ok = (z["active"] > 0.5) & (z["pressed"] < 0.5)
  return np.stack([z["t"], z["v"], z["des_angle"], z["angle"], z["des_angle"] - z["angle"], z["out"],
                   np.zeros_like(z["t"]), ok.astype(np.float64)], axis=1)


def load(path):
  if path.endswith(".jsonl"):
    return from_jsonl(path)
  if path.endswith(".npz"):
    return from_npz(path)
  return from_rlog(path)


def resample(rows):
  t = np.arange(rows[0, 0], rows[-1, 0], 1 / RATE)
  cols = [np.interp(t, rows[:, 0], rows[:, i]) for i in range(1, rows.shape[1])]
  ok = np.interp(t, rows[:, 0], rows[:, 7]) > 0.99
  return t, np.stack(cols, axis=1), ok


def lag_seconds(des, meas):
  d, m = des - des.mean(), meas - meas.mean()
  if d.std() < 1e-3 or m.std() < 1e-3:
    return float("nan")
  best, best_lag = -np.inf, 0
  for lag in range(int(RATE) + 1):  # measured lags desired by `lag` samples
    c = np.dot(d[:len(d) - lag], m[lag:]) / (len(d) - lag)
    if c > best:
      best, best_lag = c, lag
  return best_lag / RATE


def metrics(rows_list):
  """rows_list: one rows array per file; each is resampled on its own so pooling never interpolates across files."""
  parts = [resample(r) for r in rows_list]
  x = np.concatenate([p[1] for p in parts])
  ok = np.concatenate([p[2] for p in parts])
  rate = np.concatenate([np.gradient(p[1][:, 2], 1 / RATE) for p in parts])
  v, des, meas, err, out, sat = (x[:, i] for i in range(6))
  result = {}
  for name, (lo, hi) in BANDS.items():
    s = ok & (v >= lo) & (v < hi)
    if s.sum() < 5 * RATE:
      continue
    e = err[s]
    big = np.abs(e) > 0.5
    flips = np.sum(np.diff(np.sign(e[big])) != 0) if big.sum() > 1 else 0
    result[name] = {
      "seconds": round(float(s.sum() / RATE), 1),
      "rms_err": round(float(np.sqrt(np.mean(e ** 2))), 2),
      "p90_err": round(float(np.percentile(np.abs(e), 90)), 2),
      "lag_s": round(lag_seconds(des[s], meas[s]), 2),
      "osc_hz": round(float(flips / max(big.sum() / RATE, 1e-3)), 2),
      "rate_rms": round(float(np.sqrt(np.mean(rate[s] ** 2))), 1),
      "sat_frac": round(float(np.mean(sat[s] > 0.5)), 3),
      "mean_abs_out": round(float(np.mean(np.abs(out[s]))), 3),
      "mean_abs_des": round(float(np.mean(np.abs(des[s]))), 1),
    }
  return result


def table(name, m):
  for band, r in m.items():
    head = f"{name[-40:]:40s} {band:8s} {r['seconds']:6.0f}s  rms {r['rms_err']:5.2f}  p90 {r['p90_err']:5.2f}  lag {r['lag_s']:4.2f}"
    tail = f"  osc {r['osc_hz']:4.2f}/s  rate {r['rate_rms']:5.1f}  sat {r['sat_frac']:.2f}  |out| {r['mean_abs_out']:.2f}  |des| {r['mean_abs_des']:5.1f}"
    print(head + tail)


def main():
  paths = sys.argv[1:]
  allrows = []
  for p in paths:
    rows = load(p)
    if len(rows) < 100:
      print(f"{p}: no pid samples")
      continue
    table(p, metrics([rows]))
    allrows.append(rows)
  if len(allrows) > 1:
    table("POOLED", metrics(allrows))


if __name__ == "__main__":
  main()
