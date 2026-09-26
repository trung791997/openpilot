#!/usr/bin/env python3
"""Fit a steering response model (command -> wheel angle) from a car's own rlogs, for the sim's torque mode.

  tools/sim/eps_fit.py --out MODEL.json RLOG [RLOG ...]

Uses 100 Hz carOutput.actuatorsOutput.torque (the normalised command the car was sent) and carState
steering angle while openpilot steered (latActive, no driver torque, v > 5 m/s). The model is a
discrete second-order response of the angle rate, with the command delayed by `delay` frames:

  rate[k+1] = a*rate[k] + (b0 + b1*v)*u[k-delay] + (c0 + c1*v^2)*angle[k] + bias

The fit starts from one-step least squares, then refines the parameters (Levenberg-Marquardt) on the
angle error of the model free-running over 1 s windows, which is what the sim does with it. One-step
least squares alone came out far too sluggish (a 2.5 s mode): the one-step rate target is noisy, and
the fit trades response for damping. Every log but the last trains; the last is held out and scored
over 2 s free-run windows.
It captures the rack, the EPS assist and the car's self-aligning torque together. It is not a physics
model: it is only good inside the speed and command range the logs cover.
"""
import argparse
import json

import numpy as np

DT = 0.01
WINDOW = 200  # 2 s free-run windows for the held-out score


def load(path):
  from openpilot.tools.lib.logreader import LogReader
  u, rows = 0.0, []
  lat_active = False
  for m in LogReader(path):
    w = m.which()
    if w == "carOutput":
      u = m.carOutput.actuatorsOutput.torque
    elif w == "carControl":
      lat_active = m.carControl.latActive
    elif w == "carState":
      cs = m.carState
      rows.append((cs.steeringAngleDeg, cs.vEgo, u, lat_active and not cs.steeringPressed and cs.vEgo > 5.0, cs.steeringRateDeg))
  a = np.array(rows, dtype=np.float64)
  return a[:, 0], a[:, 1], a[:, 2], a[:, 3].astype(bool), a[:, 4]


def features(angle, v, u, ok, rate, delay):
  k = np.arange(delay, len(angle) - 1)
  valid = ok[k] & ok[k + 1] & ok[k - delay]
  k = k[valid]
  ud = u[k - delay]
  X = np.stack([rate[k], ud, ud * v[k], angle[k], angle[k] * v[k] ** 2, np.ones(len(k))], axis=1)
  return X, rate[k + 1]


def step(p, angle, rate, v, u_delayed):
  a, b0, b1, c0, c1, bias = p
  rate = a * rate + (b0 + b1 * v) * u_delayed + (c0 + c1 * v * v) * angle + bias
  return angle + rate * DT, rate


def windows(d, delay, length, stride):
  angle, v, u, ok, rate = d
  starts = [s for s in range(delay, len(angle) - length, stride) if ok[s - delay:s + length].all()]
  idx = np.array(starts, dtype=int).reshape(-1, 1) + np.arange(length)
  return angle[idx], rate[idx[:, 0]], v[idx], u[idx - delay]


def simulate(p, w):
  """Free-run every window in parallel from its logged angle and rate; returns the angle trajectories."""
  angle, rate0, v, u = w
  th, r = angle[:, 0].copy(), rate0.copy()
  out = np.empty_like(angle)
  for k in range(angle.shape[1]):
    out[:, k] = th
    th, r = step(p, th, r, v[:, k], u[:, k])
  return out


def refine(p, w, iters=30):
  """Levenberg-Marquardt on the free-run angle error (numerical Jacobian)."""
  lam = 1e-2
  res = (simulate(p, w) - w[0]).ravel()
  for _ in range(iters):
    J = np.empty((res.size, len(p)))
    for i in range(len(p)):
      h = 1e-6 * max(abs(p[i]), 1e-3)
      q = p.copy()
      q[i] += h
      J[:, i] = ((simulate(q, w) - w[0]).ravel() - res) / h
    A, g = J.T @ J, J.T @ res
    while True:
      q = p - np.linalg.solve(A + lam * np.diag(np.diag(A)), g)
      r2 = (simulate(q, w) - w[0]).ravel()
      if r2 @ r2 < res @ res:
        p, res, lam = q, r2, lam / 3
        break
      lam *= 10
      if lam > 1e8:
        return p
  return p


def free_run_error(p, delay, d):
  w = windows(d, delay, WINDOW, WINDOW)
  return np.abs(simulate(p, w)[:, -1] - w[0][:, -1]) if len(w[0]) else np.array([])


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--out", required=True)
  ap.add_argument("logs", nargs="+")
  args = ap.parse_args()

  data = [load(p) for p in args.logs]
  train, test = data[:-1], data[-1]
  best = None
  for delay in range(0, 31, 2):
    Xs, ys = zip(*[features(*d, delay) for d in train], strict=True)
    X, y = np.concatenate(Xs), np.concatenate(ys)
    p, *_ = np.linalg.lstsq(X, y, rcond=None)
    tw = [windows(d, delay, 100, 50) for d in train]
    p = refine(p, tuple(np.concatenate(x) for x in zip(*tw, strict=True)))
    err = free_run_error(p, delay, test)
    score = np.median(err) if len(err) else np.inf
    p90 = np.percentile(err, 90) if len(err) else np.inf
    print(f"delay {delay * DT:4.2f} s  held-out 2 s angle error median {score:5.2f} deg  p90 {p90:5.2f}  ({len(err)} windows)")
    if best is None or score < best[0]:
      best = (score, delay, p, err, len(y))
  score, delay, p, err, n = best
  model = {
    "params": dict(zip(["a", "b0", "b1", "c0", "c1", "bias"], map(float, p), strict=True)),
    "delay_frames": delay, "dt": DT,
    "train_samples": int(n), "heldout_windows": len(err),
    "heldout_2s_angle_error_deg": {"median": float(np.median(err)), "p90": float(np.percentile(err, 90))},
    "speed_range_mps": [float(min(d[1][d[3]].min() for d in data)), float(max(d[1][d[3]].max() for d in data))],
    "command_range": [float(min(d[2][d[3]].min() for d in data)), float(max(d[2][d[3]].max() for d in data))],
    "logs": [p.rsplit("/", 1)[-1] for p in args.logs],
  }
  with open(args.out, "w") as f:
    json.dump(model, f, indent=1)
  print(f"best delay {delay * DT:.2f} s; wrote {args.out}: {model['params']}")


if __name__ == "__main__":
  main()
