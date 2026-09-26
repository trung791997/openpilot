#!/usr/bin/env python3
"""Post-drive auto-tuner for the modified-EPS lateral PID speed schedule (LatGainSchedule).

Searches the P/I(/F) knots of a LatGainSchedule on the closed-loop simulator in lat_pid_sim.py and
prints a schedule for the owner to review. It never writes a param and nothing on the car reads its
output unless the owner puts it into LatGainSchedule by hand.

Method:
  1. Seed. The schedule starts from the current tuning: each knot takes the value the Lat*Scale band
     (or an existing LatGainSchedule) gives at that speed, after any --set overrides.
  2. Trust gate. Each speed band is simulated at the tuning that was actually driven and compared with
     the logged tracking. A band whose sim disagrees with the log (err rms or straight rms off by more
     than TRUST_REL, curve ratio by more than TRUST_CURVE) is untrusted: knots inside it are frozen,
     and a candidate may not make its cost any worse.
  3. Search. Best-improvement coordinate search on the free knots, steps 10 -> 5 -> 2.5 percentage
     points, within +-TRUST_REGION points of the seed and the LatGainSchedule limits. The score is
     computed on alternate 2-minute blocks of the drive ("fit"); the other blocks ("holdout") are only
     used to accept or reject the final answer.
  4. Verdict. The schedule is recommended only if the holdout cost improves, no trusted band gets worse
     on holdout by more than 2 %, and no untrusted band gets worse on all data (err rms or straight
     rms by more than 1 %, or straight sign-change rate by more than 10 %).

The default knots 20/30/40/50 mph put the whole blend inside the bands the sim can score: above 50 mph
the schedule is flat at the highway value, exactly as the bands are. Knots in an untrusted band only
move if the gate trusts that band.

Cost per band (engaged, hands off, > 4 m/s; relative to the seed, minutes-weighted across bands and routes):
  err_rms + straight_rms + 5 * |curve ratio outside 0.97..1.03| + 2 * (straight sign-change rate
  increase over the seed beyond 10 %). The last term is there because the plant under-predicts
  oscillation: the sim is only trusted to say a change makes the car less settled, never more.

Limits: sim evidence only, with the plant limits in lat_pid_sim.py (desired curvature is exogenous,
so this cannot see model-path problems; weak below 25 mph and on the highway). A recommended schedule
is a candidate for a drive, not a verified tune.

Usage:
  python tools/lateral/lat_autotune.py --plant plant.json ROUTE_DIR [ROUTE_DIR ...]
      [--knots 20,30,40,50] [--terms p,i] [--set LatPScaleStandard=115] [--out schedule.json] [-j 4]
"""
from __future__ import annotations

import argparse
import json
import multiprocessing
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lat_pid_sim as sim

TRUST_REL = 0.25
TRUST_CURVE = 0.05
TRUST_REGION = 25.0
STEPS = (10.0, 5.0, 2.5)
MIN_GAIN = 0.005
HOLDOUT_BLOCK_S = 120.0
MIN_BAND_MIN = 3.0

# Mirrors LAT_GAIN_SCHEDULE_LIMITS in selfdrive/controls/lib/latcontrol_pid.py (percent).
LIMITS = {"p": (25.0, 300.0), "i": (0.0, 300.0), "f": (0.0, 200.0)}
BAND_KEYS = {"p": ("LatPScaleLowSpeed", "LatPScaleStandard", "LatPScaleHighway", 100.0),
             "i": ("LatIScaleLowSpeed", "LatIScaleStandard", "LatIScaleHighway", 20.0),
             "f": ("LatFScaleLowSpeed", "LatFScaleStandard", "LatFScaleHighway", 100.0)}
BAND_DEFAULTS = {"p": (100.0, 100.0, 100.0), "i": (20.0, 100.0, 0.0), "f": (100.0, 100.0, 100.0)}


def band_of(v_mph):
  for name, lo, hi in sim.BANDS:
    if lo <= v_mph * sim.MPH < hi:
      return name
  return sim.BANDS[-1][0]


def seed_values(params, knots, term):
  """Percent value of `term` at each knot speed under `params` (bands, or a LatGainSchedule if set)."""
  from openpilot.selfdrive.controls.lib.latcontrol_pid import lat_gain_schedule_scale, parse_lat_gain_schedule
  sched = parse_lat_gain_schedule(params.get("LatGainSchedule"))
  keys = BAND_KEYS[term]
  bands = []
  for key, dflt in zip(keys[:3], BAND_DEFAULTS[term], strict=True):
    try:
      bands.append(float(params.get(key, dflt)))
    except (TypeError, ValueError):
      bands.append(dflt)
  out = []
  for v_mph in knots:
    v = v_mph * sim.MPH
    banded = bands[0] if v < 25 * sim.MPH else bands[1] if v < 50 * sim.MPH else bands[2]
    out.append(100.0 * lat_gain_schedule_scale(sched, term, v, banded / 100.0))
  return out


def schedule_json(knots, terms, x):
  n = len(knots)
  sched = {"v_mph": [float(k) for k in knots]}
  for j, term in enumerate(terms):
    sched[term] = [round(float(v), 1) for v in x[j * n:(j + 1) * n]]
  return json.dumps(sched, separators=(",", ":"))


# ----------------------------------------------------------------------------------------------
# evaluation (runs in worker processes; the loaded routes are inherited through fork)

_DS = []
_PLANT = None


def _masks(d):
  base = sim._hands_off(d)
  blk = (d["t"] // HOLDOUT_BLOCK_S).astype(int) % 2 == 0
  return {"fit": base & blk, "holdout": base & ~blk, "all": base}


def evaluate(overrides):
  """Per route, per mask: sim.metrics dicts."""
  res = []
  for d in _DS:
    ang, des, _, _ = sim.simulate(d, _PLANT, overrides)
    res.append({name: sim.metrics(d, ang, des, m) for name, m in _masks(d).items()})
  return res


def logged_metrics():
  return [{name: sim.metrics(d, d["angle"], d["des_angle"], m) for name, m in _masks(d).items()} for d in _DS]


def band_cost(r, r0):
  c = r["err_rms"] + (r["straight_rms"] or 0.0)
  cr = r["curve_ratio"]
  if cr is not None:
    c += 5.0 * max(0.0, 0.97 - cr) + 5.0 * max(0.0, cr - 1.03)
  if r0 is not None and r["zero_cross"] and r0["zero_cross"]:
    c += 2.0 * max(0.0, r["zero_cross"] / r0["zero_cross"] - 1.10)
  return c


def costs(res, seed_res, mask):
  """Per band: minutes-weighted cost across routes relative to the seed's (seed = 1.0), so a band with
  large absolute errors (tight turns below 25 mph) does not drown the others. Total: minutes-weighted."""
  per_band = {}
  for name, _, _ in sim.BANDS:
    num = den = w = 0.0
    for r_route, s_route in zip(res, seed_res, strict=True):
      r = r_route[mask][name]
      s = s_route[mask][name]
      if r is None or s is None:
        continue
      num += band_cost(r, s) * r["min"]
      den += band_cost(s, s) * r["min"]
      w += r["min"]
    per_band[name] = (num / den, w) if w >= MIN_BAND_MIN and den > 0 else None
  tot_w = sum(v[1] for v in per_band.values() if v is not None)
  total = sum(v[0] * v[1] for v in per_band.values() if v is not None) / tot_w if tot_w else float("nan")
  return total, per_band


def untrusted_ok(res, ref_res, trusted):
  """An untrusted band may not get worse than `ref` on any tracked figure, over all data (the fit and
  holdout halves alone can each be too short to score). Returns a list of violations."""
  bad = []
  for name, ok in trusted.items():
    if ok:
      continue
    for key, tol in (("err_rms", 1.01), ("straight_rms", 1.01), ("zero_cross", 1.10)):
      num = den = 0.0
      for r_route, f_route in zip(res, ref_res, strict=True):
        r = r_route["all"][name]
        f = f_route["all"][name]
        if r is None or f is None or r[key] is None or f[key] is None:
          continue
        num += r[key] * r["min"]
        den += f[key] * r["min"]
      if den > 0 and num > den * tol + 1e-9:
        bad.append(f"{name} {key} {num / den:.2f}x reference (untrusted band may not get worse)")
  return bad


def trust_gate(sim_res, log_res):
  """Per band: (trusted, reason). Compares the sim at the driven tuning with the log over all data."""
  out = {}
  for name, _, _ in sim.BANDS:
    s_err = l_err = s_st = l_st = 0.0
    s_cr = []
    l_cr = []
    w = 0.0
    for s_route, l_route in zip(sim_res, log_res, strict=True):
      s = s_route["all"][name]
      lg = l_route["all"][name]
      if s is None or lg is None:
        continue
      w += s["min"]
      s_err += s["err_rms"] * s["min"]
      l_err += lg["err_rms"] * s["min"]
      s_st += (s["straight_rms"] or 0.0) * s["min"]
      l_st += (lg["straight_rms"] or 0.0) * s["min"]
      if s["curve_ratio"] is not None and lg["curve_ratio"] is not None:
        s_cr.append(s["curve_ratio"])
        l_cr.append(lg["curve_ratio"])
    if w < MIN_BAND_MIN:
      out[name] = (False, f"{w:.1f} min of data")
      continue
    reasons = []
    if abs(s_err - l_err) > TRUST_REL * l_err:
      reasons.append(f"err rms sim {s_err / w:.2f} vs log {l_err / w:.2f}")
    if l_st > 0 and abs(s_st - l_st) > TRUST_REL * l_st:
      reasons.append(f"straight rms sim {s_st / w:.2f} vs log {l_st / w:.2f}")
    if s_cr and abs(np.mean(s_cr) - np.mean(l_cr)) > TRUST_CURVE:
      reasons.append(f"curve ratio sim {np.mean(s_cr):.3f} vs log {np.mean(l_cr):.3f}")
    out[name] = (not reasons, "; ".join(reasons) or f"sim matches log ({w:.1f} min)")
  return out


# ----------------------------------------------------------------------------------------------

def search(pool, base_overrides, knots, terms, x0, free, lo, hi, seed_res, trusted, verbose=True):
  def overrides_for(x):
    o = dict(base_overrides)
    o["LatGainSchedule"] = schedule_json(knots, terms, x)
    return o

  def score(res):
    if untrusted_ok(res, seed_res, trusted):
      return float("inf")
    return costs(res, seed_res, "fit")[0]

  x = np.array(x0, dtype=float)
  best = score(seed_res)
  evals = 0
  for step in STEPS:
    while True:
      cands = []
      for j in np.flatnonzero(free):
        for sgn in (-1.0, 1.0):
          xn = x.copy()
          xn[j] = min(max(xn[j] + sgn * step, lo[j]), hi[j])
          if xn[j] != x[j]:
            cands.append(xn)
      if not cands:
        break
      results = pool.map(evaluate, [overrides_for(c) for c in cands])
      evals += len(cands)
      scores = [score(r) for r in results]
      k = int(np.argmin(scores))
      if scores[k] < best * (1.0 - MIN_GAIN):
        x, best = cands[k], scores[k]
        if verbose:
          print(f"  step {step:4.1f}  fit cost {best:.4f}  {schedule_json(knots, terms, x)}", flush=True)
      else:
        break
  if verbose:
    print(f"  search done: {evals} closed-loop evaluations", flush=True)
  return x, best


def _band_table(label, per_band):
  cells = []
  for name, _, _ in sim.BANDS:
    v = per_band[name]
    cells.append(f"{name}: {'  -  ' if v is None else format(v[0], '.4f')}")
  return f"    {label:22s} " + "  |  ".join(cells)


def main(argv=None):
  global _DS, _PLANT
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--plant", required=True, help="plant json from lat_pid_sim.py fit")
  ap.add_argument("--knots", default="20,30,40,50", help="knot speeds, mph, strictly increasing (2-8)")
  ap.add_argument("--terms", default="p,i", help="subset of p,i,f to tune")
  ap.add_argument("--set", action="append", help="current tuning if it differs from the logged one, K=V")
  ap.add_argument("--trust-region", type=float, default=TRUST_REGION, help="max move per knot, percentage points")
  ap.add_argument("--out", help="write the result as JSON here")
  ap.add_argument("-j", "--jobs", type=int, default=min(4, os.cpu_count() or 1))
  args = ap.parse_args(argv)

  knots = [float(k) for k in args.knots.split(",")]
  terms = [t.strip() for t in args.terms.split(",") if t.strip()]
  if not 2 <= len(knots) <= 8 or any(b <= a for a, b in zip(knots[:-1], knots[1:], strict=True)):
    ap.error("--knots must be 2-8 strictly increasing speeds")
  if not terms or any(t not in LIMITS for t in terms):
    ap.error("--terms must be a subset of p,i,f")

  _DS = sim.load(args.routes)
  with open(args.plant) as f:
    _PLANT = sim.Plant.from_json(json.load(f))
  user = sim._overrides(args.set)
  for d in _DS:
    if "LatGainSchedule" in d["params"] and "LatGainSchedule" not in user:
      print(f"note: {d['route']} was driven with LatGainSchedule {d['params']['LatGainSchedule']}")

  # The seed and the search start from the owner's current tuning: the logged params of the first
  # route plus --set. Each route is still simulated with its own logged params underneath.
  current = dict(_DS[0]["params"])
  current.update(user)
  x0 = []
  lo = []
  hi = []
  for term in terms:
    vals = seed_values(current, knots, term)
    x0 += vals
    lo += [max(LIMITS[term][0], v - args.trust_region) for v in vals]
    hi += [min(LIMITS[term][1], v + args.trust_region) for v in vals]
  seed_overrides = dict(user)
  seed_overrides["LatGainSchedule"] = schedule_json(knots, terms, x0)

  ctx = multiprocessing.get_context("fork")
  with ctx.Pool(args.jobs) as pool:
    print("== evaluating the driven tuning, the current tuning and the seed schedule", flush=True)
    driven_res, current_res, seed_res = pool.map(evaluate, [{}, user, seed_overrides])
    log_res = logged_metrics()

    gate = trust_gate(driven_res, log_res)
    trusted = {name: ok for name, (ok, _) in gate.items()}
    print("  sim vs log at the driven tuning:")
    for name, (ok, why) in gate.items():
      print(f"    {name:15s} {'trusted  ' if ok else 'UNTRUSTED'}  {why}")
    free = np.array([trusted[band_of(k)] for _ in terms for k in knots])
    if not free.any():
      print("no band passes the trust gate: nothing to tune. The plant needs refitting on data like this.")
      return 1

    frozen = [f"{t}@{k:g}" for (t, k), f in zip(((t, k) for t in terms for k in knots), free, strict=True) if not f]
    print(f"  seed   {schedule_json(knots, terms, x0)}")
    if frozen:
      print(f"  frozen knots (untrusted band): {', '.join(frozen)}")
    print("== search", flush=True)
    x, _ = search(pool, user, knots, terms, x0, free, lo, hi, seed_res, trusted)
    prop_overrides = dict(user)
    prop_overrides["LatGainSchedule"] = schedule_json(knots, terms, x)
    (prop_res,) = pool.map(evaluate, [prop_overrides])

  print("== result (sim evidence only)")
  verdict = True
  reasons = []
  for mask in ("fit", "holdout"):
    print(f"  {mask} cost per band, relative to the seed schedule (lower is better)")
    c_tot, c_band = costs(current_res, seed_res, mask)
    s_tot, s_band = costs(seed_res, seed_res, mask)
    p_tot, p_band = costs(prop_res, seed_res, mask)
    print(_band_table("current bands", c_band) + f"  | total {c_tot:.4f}")
    print(_band_table("seed schedule", s_band) + f"  | total {s_tot:.4f}")
    print(_band_table("proposed schedule", p_band) + f"  | total {p_tot:.4f}")
    if mask == "holdout":
      if not p_tot < c_tot:
        verdict = False
        reasons.append("holdout total does not improve on the current bands")
      for name, ok in trusted.items():
        if p_band[name] is None or c_band[name] is None:
          continue
        if not ok:
          continue  # checked on all data below
        limit = c_band[name][0] * 1.02 + 1e-9
        if p_band[name][0] > limit:
          verdict = False
          reasons.append(f"{name} holdout worse than current ({p_band[name][0]:.4f} vs {c_band[name][0]:.4f})")

  bad = untrusted_ok(prop_res, current_res, trusted)
  if bad:
    verdict = False
    reasons += bad

  print("  proposed vs current, all data, per band:")
  for name, _, _ in sim.BANDS:
    rows = []
    for label, res in (("current ", current_res), ("proposed", prop_res)):
      ms = [r["all"][name] for r in res if r["all"][name] is not None]
      if not ms:
        continue
      w = sum(m["min"] for m in ms)
      crs = [m["curve_ratio"] for m in ms if m["curve_ratio"] is not None]
      zcs = [m["zero_cross"] for m in ms if m["zero_cross"] is not None]
      rows.append(f"      {label}  err rms {sum(m['err_rms'] * m['min'] for m in ms) / w:.3f}  " +
                  f"straight rms {sum((m['straight_rms'] or 0) * m['min'] for m in ms) / w:.3f}  " +
                  f"curve {np.mean(crs) if crs else float('nan'):.3f}  sign changes {np.mean(zcs) if zcs else float('nan'):.2f}/s")
    if rows:
      print(f"    {name}{'' if trusted[name] else '  (untrusted: frozen, may not get worse)'}")
      print("\n".join(rows))

  sched = schedule_json(knots, terms, x)
  print()
  if verdict and not np.allclose(x, x0):
    print("RECOMMENDED (sim evidence; drive it and compare before keeping it):")
    print(f"  LatGainSchedule = {sched}")
  else:
    print("NO CHANGE RECOMMENDED: " + ("; ".join(reasons) if reasons else "search found nothing better than the seed"))
    print(f"  (candidate was {sched})")
  if args.out:
    with open(args.out, "w") as f:
      json.dump({"LatGainSchedule": sched, "recommended": bool(verdict and not np.allclose(x, x0)), "reasons": reasons,
                 "seed": schedule_json(knots, terms, x0), "routes": [d["route"] for d in _DS],
                 "trust_gate": {k: {"trusted": ok, "why": why} for k, (ok, why) in gate.items()}, "set": user}, f, indent=2)
  return 0


if __name__ == "__main__":
  sys.exit(main())
