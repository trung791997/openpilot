"""Closed-loop sim step of the NRDR PID Tuning trial (Galaxy page and lat_tune_cli).

lat_tune_analyzer turns the logs into a rule-based P step per speed band. This module re-scores each band on
lat_pid_sim with a plant fitted to the car's own EPS image, and where the sim reproduces that band's logged
tracking it replaces the rule step with the best P/I pair from a small grid around the driven values.

Per band (the owner's three: LowSpeed < 25 mph, Standard 25-50, Highway 50+):
  1. Trust. The band is simulated at the tuning it was driven with and compared with the log, with the same
     gate as lat_autotune (err rms and straight rms within TRUST_REL, curve ratio within TRUST_CURVE, at least
     MIN_BAND_MIN minutes). An untrusted band keeps the rule result.
  2. Grid. P in {-P_STEP, 0, +P_STEP} % of the driven value (5 % grid) x I in {-I_STEP, 0, +I_STEP} points.
     The bands switch hard at 25/50 mph, so one closed-loop run sets all three bands at once and 9 runs score
     every band. I is held when LatGainSchedule overrides it (the band key would not reach the controller).
  3. Pick. Cost = err rms + straight rms + 5 x |curve ratio outside 0.97..1.03| + 2 x (straight sign-change
     rate rise beyond 10 %), relative to the driven pair, scored on alternate 2-minute blocks (fit); the other
     blocks (holdout) must agree. A pair is proposed only if it beats the driven pair by MIN_GAIN on both, its
     sign-change rate stays under the analyzer's SIGN_RATE_MAX, and the logs do not veto a step up (straight
     sign changes over SIGN_RATE_UP_MAX or override onsets over PRESS_RATE_UP_MAX, the rule engine's vetoes).

The plant is only valid for the image it was fitted on, so the step is skipped (rules only) unless the routes
are from the car and EPS firmware named in the plant file. A reflash with a different table or tracker keeps
the firmware string; the trust gate is what catches that (the sim stops matching the log).

Sim evidence only: nothing here has been driven. A proposal is a candidate for the next drive.
"""
import json
import multiprocessing
import os
import time


from openpilot.selfdrive.controls.lib import lat_tune_analyzer as lat
from openpilot.tools.lateral import lat_pid_sim as sim

PLANT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plants")
DEFAULT_PLANT = os.path.join(PLANT_DIR, "civic_bosch_c020.json")

# Same gate and cost as lat_autotune (tests hold them equal).
TRUST_REL = 0.25
TRUST_CURVE = 0.05
MIN_BAND_MIN = 3.0
HOLDOUT_BLOCK_S = 120.0
MIN_GAIN = 0.02            # a pair must beat the driven one by 2 % on fit and on holdout

P_STEP = 0.10              # fraction of the driven P, rounded to the 5 % grid
I_STEP = 25                # percentage points
PCT_MIN, PCT_MAX = lat.PCT_MIN, lat.PCT_MAX
# Engaged minutes simulated per trial, newest routes first. One run is ~0.7 s per route minute on a desktop
# core; the trial needs 9 runs, so 120 min is ~13 min of desktop time, and several times that on the device.
MAX_SIM_MIN = 120.0

SIM_BAND = {lat.BAND_NAMES[i]: name for i, (name, _, _) in enumerate(sim.BANDS)}


# ---------------------------------------------------------------- plant

def load_plant(path=DEFAULT_PLANT):
  """(plant, meta). meta: the plant file minus the coefficients ("car", "eps_fw", "image", "fitted_on", ...)."""
  with open(path) as f:
    j = json.load(f)
  meta = {k: v for k, v in j.items() if k not in ("bands", "coef", "eps")}
  meta["path"] = os.path.relpath(path, os.path.dirname(PLANT_DIR))
  return sim.Plant.from_json(j), meta


def plant_matches(cp_bytes, meta):
  """(ok, reason): the route's car and EPS firmware are the ones the plant was fitted on."""
  from cereal import car
  with car.CarParams.from_bytes(cp_bytes) as cp:
    fingerprint = str(cp.carFingerprint)
    eps = sorted({bytes(f.fwVersion).rstrip(b"\x00").decode("latin-1") for f in cp.carFw if str(f.ecu) == "eps"})
  if meta.get("car") and fingerprint != meta["car"]:
    return False, f"car {fingerprint}, plant fitted on {meta['car']}"
  want = meta.get("eps_fw") or []
  if want and not any(e in want for e in eps):
    return False, f"EPS firmware {', '.join(eps) or 'unknown'}, plant fitted on {', '.join(want)}"
  return True, f"{fingerprint}, EPS {', '.join(e for e in eps if e in want) or 'any'}"


# ---------------------------------------------------------------- grid

def candidate_gains(gains, schedule_terms):
  """[(dp, di)] -> per-band {"p", "i"}; (0, 0) is the driven pair. gains: analyzer band_gains()."""
  out = {}
  for dp in (-1, 0, 1):
    for di in (-1, 0, 1):
      if di and "i" in schedule_terms:
        continue
      row = []
      for g in gains:
        p = g["p"] if not dp else int(round(g["p"] * (1 + dp * P_STEP) / lat.PCT_STEP)) * lat.PCT_STEP
        if dp and p == g["p"]:
          p = g["p"] + dp * lat.PCT_STEP
        row.append({"p": min(max(p, max(PCT_MIN, lat.PCT_STEP)), PCT_MAX), "i": min(max(g["i"] + di * I_STEP, 0), PCT_MAX)})
      out[(dp, di)] = row
  return out


def overrides_for(row, schedule_raw):
  """Param overrides for one candidate: the band keys, plus LatGainSchedule without its p term (apply strips it,
  and while it is there it overrides the P bands)."""
  o = {}
  for n, g in enumerate(row):
    o[lat.P_KEYS[n]] = str(g["p"])
    o[lat.I_KEYS[n]] = str(g["i"])
  if lat._param_str(schedule_raw).strip():
    o[lat.SCHEDULE_KEY] = lat.strip_schedule_p(schedule_raw)
  return o


# ---------------------------------------------------------------- scoring

def _masks(d):
  base = sim._hands_off(d)
  blk = (d["t"] // HOLDOUT_BLOCK_S).astype(int) % 2 == 0
  return {"fit": base & blk, "holdout": base & ~blk, "all": base}


def _score(d, ang, des):
  return {name: sim.metrics(d, ang, des, m) for name, m in _masks(d).items()}


_JOB = {}


def _run(args):
  ri, key = args
  d = _JOB["ds"][ri]
  ang, des, _, _ = sim.simulate(d, _JOB["plant"], _JOB["overrides"][key])
  return ri, key, _score(d, ang, des)


def band_cost(r, r0):
  c = r["err_rms"] + (r["straight_rms"] or 0.0)
  cr = r["curve_ratio"]
  if cr is not None:
    c += 5.0 * max(0.0, 0.97 - cr) + 5.0 * max(0.0, cr - 1.03)
  if r0 is not None and r.get("sign_hyst") and r0.get("sign_hyst"):
    c += 2.0 * max(0.0, r["sign_hyst"] / r0["sign_hyst"] - 1.10)
  return c


def pooled(per_route, mask, band, keys=("err_rms", "straight_rms", "curve_ratio", "sign_hyst")):
  """Minutes-weighted figures over routes for one band and mask; None below MIN_BAND_MIN minutes."""
  rows = [r[mask][band] for r in per_route if r[mask][band] is not None]
  w = sum(r["min"] for r in rows)
  if w < MIN_BAND_MIN:
    return None
  out = {"min": round(w, 2)}
  for k in keys:
    vals = [(r[k], r["min"]) for r in rows if r[k] is not None]
    out[k] = round(sum(v * m for v, m in vals) / sum(m for _, m in vals), 4) if vals else None
  return out


def rel_cost(res, base, mask, band):
  num = den = 0.0
  for r_route, b_route in zip(res, base, strict=True):
    r, b = r_route[mask][band], b_route[mask][band]
    if r is None or b is None:
      continue
    num += band_cost(r, b) * r["min"]
    den += band_cost(b, b) * r["min"]
  return num / den if den > 0 else None


def trust(sim_res, log_res, band):
  s, lg = pooled(sim_res, "all", band), pooled(log_res, "all", band)
  if s is None or lg is None:
    w = s["min"] if s else 0.0
    return False, f"{w:.1f} min of data (needs {MIN_BAND_MIN:g})"
  reasons = []
  if abs(s["err_rms"] - lg["err_rms"]) > TRUST_REL * lg["err_rms"]:
    reasons.append(f"err rms sim {s['err_rms']:.2f} vs log {lg['err_rms']:.2f}")
  if lg["straight_rms"] and abs(s["straight_rms"] - lg["straight_rms"]) > TRUST_REL * lg["straight_rms"]:
    reasons.append(f"straight rms sim {s['straight_rms']:.2f} vs log {lg['straight_rms']:.2f}")
  if s["curve_ratio"] is not None and lg["curve_ratio"] is not None and abs(s["curve_ratio"] - lg["curve_ratio"]) > TRUST_CURVE:
    reasons.append(f"curve ratio sim {s['curve_ratio']:.3f} vs log {lg['curve_ratio']:.3f}")
  return not reasons, "; ".join(reasons) or f"sim matches log ({s['min']:.1f} min)"


def _pick(n, band, results, cands, base_key, trial_band):
  """(key, note) for one band: the best candidate, or base_key with why nothing better was taken."""
  base = results[base_key]
  scored = []
  for key, res in results.items():
    if key == base_key:
      continue
    fit, hold = rel_cost(res, base, "fit", band), rel_cost(res, base, "holdout", band)
    if fit is None or hold is None:
      continue
    scored.append((fit, hold, key))
  scored.sort()
  cur = cands[base_key][n]
  for fit, hold, key in scored:
    if fit > 1.0 - MIN_GAIN:
      break
    g = cands[key][n]
    if hold > 1.0 - MIN_GAIN:
      continue
    s = pooled(results[key], "all", band)
    if s and s["sign_hyst"] is not None and s["sign_hyst"] > lat.SIGN_RATE_MAX:
      continue
    up = g["p"] > cur["p"] or g["i"] > cur["i"]
    if up and (trial_band.get("pressRate") or 0.0) > lat.PRESS_RATE_UP_MAX:
      continue
    if g["p"] > cur["p"] and (trial_band.get("signRate") or 0.0) > lat.SIGN_RATE_UP_MAX:
      continue
    return key, f"fit {fit:.3f}, holdout {hold:.3f} of the driven pair"
  best = f" (best fit {scored[0][0]:.3f}, holdout {scored[0][1]:.3f})" if scored else ""
  return base_key, f"no pair beats the driven one by {MIN_GAIN:.0%} on fit and holdout within the vetoes{best}"


def _choose_routes(ds, max_min):
  """Newest first until max_min engaged minutes; returned oldest first."""
  keep, total = [], 0.0
  for d in reversed(ds):
    m = float(sim._hands_off(d).sum()) / 100 / 60
    if keep and total + m > max_min:
      continue
    keep.append(d)
    total += m
  return keep[::-1], total


def refine_trial(trial, ds, plant, meta, workers=1, should_continue=None, on_progress=None, max_min=MAX_SIM_MIN):
  """Adds trial["sim"] and per-band "sim" entries; a trusted band whose grid finds a better pair gets
  source "sim" with that pair as its proposal. ds: lat_pid_sim.extract dicts of the trial's used routes."""
  t0 = time.monotonic()
  info = {"plant": meta.get("path"), "image": meta.get("image"), "fittedOn": meta.get("fitted_on_summary")}
  trial["sim"] = info
  if not ds:
    info["status"] = "skipped: no route data"
    return trial
  ok, why = plant_matches(ds[-1]["cp_bytes"], meta)
  info["match"] = why
  if not ok:
    info["status"] = f"skipped: {why}"
    return trial
  ds, minutes = _choose_routes(ds, max_min)
  info["routes"] = [d["route"] for d in ds]
  info["minutes"] = round(minutes, 1)
  gains = trial["baseline"]["gains"]
  schedule_raw = trial["baseline"]["raw"].get(lat.SCHEDULE_KEY, "")
  cands = candidate_gains(gains, trial["baseline"].get("scheduleTerms") or [])
  base_key = (0, 0)
  _JOB.update(ds=ds, plant=plant, overrides={k: overrides_for(row, schedule_raw) for k, row in cands.items()})
  jobs = [(ri, key) for key in cands for ri in range(len(ds))]
  results = {key: [None] * len(ds) for key in cands}
  try:
    if workers > 1:
      ctx = multiprocessing.get_context("fork")
      with ctx.Pool(workers) as pool:
        for done, (ri, key, res) in enumerate(pool.imap_unordered(_run, jobs), 1):
          results[key][ri] = res
          if on_progress is not None:
            on_progress(done, len(jobs))
          if should_continue is not None and not should_continue():
            pool.terminate()
            raise InterruptedError("stopped")
    else:
      for done, job in enumerate(jobs, 1):
        if should_continue is not None and not should_continue():
          raise InterruptedError("stopped")
        ri, key, res = _run(job)
        results[key][ri] = res
        if on_progress is not None:
          on_progress(done, len(jobs))
  finally:
    _JOB.clear()
  log_res = [_score(d, d["angle"], d["des_angle"]) for d in ds]

  for n, b in enumerate(trial["bands"]):
    band = SIM_BAND[b["name"]]
    trusted, reason = trust(results[base_key], log_res, band)
    entry = {"trusted": trusted, "trust": reason, "log": pooled(log_res, "all", band),
             "simDriven": pooled(results[base_key], "all", band)}
    b["sim"] = entry
    b.setdefault("source", "rules")
    if not trusted:
      entry["note"] = "untrusted: the rule result stands"
      continue
    key, note = _pick(n, band, results, cands, base_key, b)
    entry["note"] = note
    entry["grid"] = [{"p": cands[k][n]["p"], "i": cands[k][n]["i"], "fit": _r(rel_cost(results[k], results[base_key], "fit", band)),
                      "holdout": _r(rel_cost(results[k], results[base_key], "holdout", band))} for k in cands]
    cur, new = b["current"], cands[key][n]
    b["source"] = "sim"
    b["proposed"] = {"p": int(new["p"]), "i": int(new["i"]), "f": cur["f"]}
    b["factor"] = round(new["p"] / cur["p"], 4) if cur["p"] else 1.0
    b["iDelta"] = int(new["i"] - cur["i"])
    if key == base_key:
      b["decision"] = "hold"
      b["reason"] = f"sim: hold ({note})"
    else:
      entry["simProposed"] = pooled(results[key], "all", band)
      moves = [f"P {cur['p']} -> {new['p']}" if new["p"] != cur["p"] else "", f"I {cur['i']} -> {new['i']}" if new["i"] != cur["i"] else ""]
      b["decision"] = "sim"
      b["reason"] = f"sim: {', '.join(m for m in moves if m)} ({note})"
  info["status"] = "used"
  info["seconds"] = round(time.monotonic() - t0, 1)
  return trial


def _r(x):
  return None if x is None else round(x, 4)
