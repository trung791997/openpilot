"""Galaxy Plots: a 20 Hz tracking recorder for lateral and longitudinal control.

What it compares, and why those signals:
  lateral       desiredCurvature * v^2  vs  curvature * v^2   (controlsState, m/s^2)
  longitudinal  longitudinalPlan.aTarget vs carState.aEgo     (m/s^2; aEgo is what longcontrol closes on)
Only samples where openpilot is actually in control are scored: carControl.latActive with no steering
touch, and carControl.longActive with no gas press in the pid/starting states. The old sampler read
controlsState.active / controlsState.aTarget, which do not exist in this schema, and polled at 1.3 Hz.

Sampling is paced by longitudinalPlan (20 Hz) with every other service conflated to its latest message,
so a full drive costs five small deserialisations per 50 ms. 20 Hz resolves up to 10 Hz, which covers
steering ping-pong (1-3 Hz) and lets the response lag be measured to 50 ms.

The analysis is a heuristic summary of one drive, not a verdict on a tune. Everything it says is bound
to the recorded samples and the tune snapshot stored beside them.
"""
import csv
import gzip
import io
import json
import os
import shutil
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

COLUMNS = [
  "t", "v", "a_ego", "enabled", "lat_active", "long_active", "steer_pressed", "gas_pressed", "brake_pressed",
  "lat_des", "lat_act", "long_des", "long_act", "long_state",
  "lat_p", "lat_i", "lat_d", "lat_f", "long_up", "long_ui", "long_uf", "lat_sat",
]
COL = {name: i for i, name in enumerate(COLUMNS)}
BOOL_COLUMNS = {"enabled", "lat_active", "long_active", "steer_pressed", "gas_pressed", "brake_pressed", "lat_sat"}

LONG_STATES = {"off": 0, "pid": 1, "stopping": 2, "starting": 3}

LIVE_BUFFER_S = 60.0
LIVE_WINDOW_S = 30.0
NOMINAL_DT = 0.05
CLIENT_IDLE_TIMEOUT_S = 6.0
OFFROAD_AUTOSTOP_S = 30.0
MAX_RECORDING_S = 8 * 3600.0   # ~90 MB of CSV; stops a forgotten recording from filling /data
FLUSH_EVERY_ROWS = 20

# ---- analysis thresholds (heuristics; each finding that uses one says so) ----
SEGMENT_GAP_S = 0.2          # a gap longer than this splits the series; lagged pairs never cross it
MAX_LAG_S = 1.5
LAT_MIN_SPEED = 5.0          # m/s; below this lateral accel is too small to say anything
LAT_CURVE_DEMAND = 0.5       # m/s^2; "in a curve" for the gain estimate
LAT_STRAIGHT_DEMAND = 0.15   # m/s^2; "straight" for the drift estimate
LONG_REGIME_DEMAND = 0.25    # m/s^2; accel / braking regimes for the bias estimate
LONG_GAIN_DEMAND = 0.3
HARD_BRAKE = -2.5            # m/s^2
GAIN_LOW, GAIN_HIGH = 0.85, 1.15
WOBBLE_RATIO_HIGH = 1.5
WOBBLE_FLOOR = 0.02          # m/s^2; keeps the ratio finite on a dead-straight road
LAT_BIAS_NOTABLE = 0.05
LONG_BIAS_NOTABLE = 0.15
JERK_HIGH = 2.5              # m/s^3, p95 of engaged actual jerk
LAT_RMSE_GOOD, LAT_RMSE_FAIR = 0.20, 0.40
LONG_RMSE_GOOD, LONG_RMSE_FAIR = 0.25, 0.45
SATURATION_NOTABLE = 0.05    # fraction of curve samples at the steering limit
BAND_MIN_S = 10.0            # a speed band needs this much engaged time to be reported
BAND_GAIN_SPREAD = 0.15      # gain difference between bands worth a sentence
MPH = 2.23694
# Speed bands for the per-band breakdown, in m/s (edges are round in mph: 30 / 50 / 70).
SPEED_BANDS = [(0.0, 13.41), (13.41, 22.35), (22.35, 31.29), (31.29, float("inf"))]


# =====================================================================================================
# Analysis (pure; unit-tested on synthetic signals)
# =====================================================================================================

def _as_arrays(rows):
  """rows: list of sequences in COLUMNS order, or an (N, len(COLUMNS)) array."""
  data = np.asarray(rows, dtype=float).reshape(-1, len(COLUMNS))
  return {name: data[:, i] for i, name in enumerate(COLUMNS)}


def _segments(t):
  if len(t) == 0:
    return np.zeros(0, dtype=int)
  return np.concatenate([[0], np.cumsum(np.diff(t) > SEGMENT_GAP_S)])


def _dt(t):
  if len(t) < 2:
    return NOMINAL_DT
  d = np.diff(t)
  d = d[(d > 0) & (d <= SEGMENT_GAP_S)]
  return float(np.median(d)) if len(d) else NOMINAL_DT


def _rising_edges(x):
  x = x.astype(bool)
  if len(x) < 2:
    return 0
  return int(np.count_nonzero(x[1:] & ~x[:-1]))


def _runs(mask, seg, min_len):
  """Contiguous index runs where mask is true inside one segment."""
  runs = []
  start = None
  for i in range(len(mask)):
    ok = bool(mask[i]) and (start is None or seg[i] == seg[start])
    if ok and start is None:
      start = i
    elif not ok and start is not None:
      if i - start >= min_len:
        runs.append((start, i))
      start = i if mask[i] else None
  if start is not None and len(mask) - start >= min_len:
    runs.append((start, len(mask)))
  return runs


def _moving_average(x, n):
  if n <= 1 or len(x) < n:
    return x.copy()
  return np.convolve(x, np.ones(n) / n, mode="same")


def _best_lag(des, act, mask, seg, dt):
  """Shift actual back by k samples (actual trails target) and keep the k with the lowest RMSE.
  Pairs are only formed when both ends are scored samples in the same segment."""
  n = len(des)
  best = None
  rmse0 = None
  for k in range(0, int(round(MAX_LAG_S / dt)) + 1):
    if k >= n:
      break
    i = np.arange(n - k)
    ok = mask[i] & mask[i + k] & (seg[i] == seg[i + k])
    if np.count_nonzero(ok) < 20:
      continue
    d = des[i][ok]
    a = act[i + k][ok]
    rmse = float(np.sqrt(np.mean((a - d) ** 2)))
    if k == 0:
      rmse0 = rmse
    if best is None or rmse < best[1] - 1e-9:
      best = (k, rmse, d, a, i[ok])
  if best is None:
    return None
  k, rmse, d, a, idx = best
  return {"lag_s": k * dt, "rmse": rmse, "rmse_no_lag": rmse0 if rmse0 is not None else rmse, "des": d, "act": a, "idx": idx}


def _gain(d, a, demand):
  sel = np.abs(d) > demand
  if np.count_nonzero(sel) < 40:
    return None
  return float(np.sum(d[sel] * a[sel]) / np.sum(d[sel] ** 2))


def _mean_where(x, sel, min_count=40):
  if np.count_nonzero(sel) < min_count:
    return None
  return float(np.mean(x[sel]))


def _r(x, digits=3):
  return None if x is None else round(float(x), digits)


def _speed_bands(v, d, a, dt, gain_demand):
  """Tracking error and gain per speed band, on lag-aligned pairs. Bands with little data are left out."""
  out = []
  err = a - d
  for lo, hi in SPEED_BANDS:
    sel = (v >= lo) & (v < hi)
    n = int(np.count_nonzero(sel))
    if n * dt < BAND_MIN_S:
      continue
    out.append({
      "lo_ms": lo, "hi_ms": None if hi == float("inf") else hi,
      "engaged_s": round(n * dt, 1),
      "rmse": _r(np.sqrt(np.mean(err[sel] ** 2))),
      "gain": _r(_gain(d[sel], a[sel], gain_demand)),
      "bias": _r(np.mean(err[sel])),
    })
  return out


def _band_label(b):
  lo, hi = b["lo_ms"] * MPH, None if b["hi_ms"] is None else b["hi_ms"] * MPH
  if hi is None:
    return f"{lo:.0f}+ mph"
  if lo == 0:
    return f"under {hi:.0f} mph"
  return f"{lo:.0f}-{hi:.0f} mph"


def _analyze_lateral(c, seg, dt, min_engaged_s):
  mask = (c["lat_active"] > 0.5) & (c["steer_pressed"] < 0.5) & (c["v"] > LAT_MIN_SPEED)
  engaged_s = float(np.count_nonzero(mask) * dt)
  out = {
    "engaged_s": round(engaged_s, 1),
    "steer_overrides": _rising_edges((c["steer_pressed"] > 0.5) & (c["lat_active"] > 0.5)),
  }
  if engaged_s < min_engaged_s:
    out["status"] = "insufficient"
    return out
  fit = _best_lag(c["lat_des"], c["lat_act"], mask, seg, dt)
  if fit is None:
    out["status"] = "insufficient"
    return out
  d, a = fit["des"], fit["act"]
  err = a - d
  hp_act, hp_des = [], []
  win = max(3, int(round(1.0 / dt)) | 1)
  for s, e in _runs(mask, seg, win * 2):
    for src, dst in ((c["lat_act"], hp_act), (c["lat_des"], hp_des)):
      x = src[s:e]
      hp = (x - _moving_average(x, win))[win // 2: len(x) - win // 2]
      dst.append(hp)
  wobble = None
  if hp_act:
    ra = float(np.sqrt(np.mean(np.concatenate(hp_act) ** 2)))
    rd = float(np.sqrt(np.mean(np.concatenate(hp_des) ** 2)))
    wobble = ra / max(rd, WOBBLE_FLOOR)
  in_curve = np.abs(d) > LAT_CURVE_DEMAND
  sat = c["lat_sat"][fit["idx"]] > 0.5
  saturated_curve = float(np.mean(sat[in_curve])) if np.count_nonzero(in_curve) >= 40 else None
  out.update({
    "status": "ok",
    "lag_s": _r(fit["lag_s"], 2),
    "rmse": _r(fit["rmse"]),
    "rmse_no_lag": _r(fit["rmse_no_lag"]),
    "p95_abs_error": _r(np.percentile(np.abs(err), 95)),
    "curve_gain": _r(_gain(d, a, LAT_CURVE_DEMAND)),
    "straight_bias": _r(_mean_where(err, np.abs(d) < LAT_STRAIGHT_DEMAND, 100)),
    "wobble_ratio": _r(wobble, 2),
    "peak_demand": _r(np.max(np.abs(d))),
    "saturated_curve_frac": _r(saturated_curve),
    "speed_bands": _speed_bands(c["v"][fit["idx"]], d, a, dt, LAT_CURVE_DEMAND),
  })
  return out


def _analyze_longitudinal(c, seg, dt, min_engaged_s):
  state = np.rint(c["long_state"]).astype(int)
  mask = ((c["long_active"] > 0.5) & (c["gas_pressed"] < 0.5) &
          np.isin(state, [LONG_STATES["pid"], LONG_STATES["starting"]]) &
          ~((c["v"] < 0.3) & (c["long_des"] <= 0.0)))
  engaged_s = float(np.count_nonzero(mask) * dt)
  out = {
    "engaged_s": round(engaged_s, 1),
    "gas_overrides": _rising_edges((c["gas_pressed"] > 0.5) & (c["enabled"] > 0.5)),
    "hard_brakes": _rising_edges((c["long_act"] < HARD_BRAKE) & (c["long_active"] > 0.5)),
  }
  if engaged_s < min_engaged_s:
    out["status"] = "insufficient"
    return out
  fit = _best_lag(c["long_des"], c["long_act"], mask, seg, dt)
  if fit is None:
    out["status"] = "insufficient"
    return out
  d, a = fit["des"], fit["act"]
  err = a - d
  jerks = []
  smooth = max(1, int(round(0.25 / dt)))
  for s, e in _runs(mask, seg, smooth * 4):
    x = _moving_average(c["long_act"][s:e], smooth)[smooth: -smooth or None]
    if len(x) > 2:
      jerks.append(np.abs(np.diff(x)) / dt)
  out.update({
    "status": "ok",
    "lag_s": _r(fit["lag_s"], 2),
    "rmse": _r(fit["rmse"]),
    "rmse_no_lag": _r(fit["rmse_no_lag"]),
    "p95_abs_error": _r(np.percentile(np.abs(err), 95)),
    "gain": _r(_gain(d, a, LONG_GAIN_DEMAND)),
    "accel_bias": _r(_mean_where(err, d > LONG_REGIME_DEMAND)),
    "brake_bias": _r(_mean_where(err, d < -LONG_REGIME_DEMAND)),
    "jerk_p95": _r(np.percentile(np.concatenate(jerks), 95), 2) if jerks else None,
    "speed_bands": _speed_bands(c["v"][fit["idx"]], d, a, dt, LONG_GAIN_DEMAND),
  })
  return out


def _fmt_time(seconds):
  seconds = int(round(seconds or 0))
  if seconds < 60:
    return f"{seconds} s"
  m, sec = divmod(seconds, 60)
  if m < 60:
    return f"{m} min {sec:02d} s"
  h, m = divmod(m, 60)
  return f"{h} h {m:02d} min"


def _feel(rmse, good, fair):
  if rmse <= good:
    return "small enough that you would hardly feel it"
  if rmse <= fair:
    return "enough that you would notice it in places"
  return "large enough to feel most of the time"


def _band_notes(bands, what, takeaways):
  known = [b for b in (bands or []) if b.get("gain") is not None]
  if len(known) < 2:
    return []
  parts = ", ".join(f"{_band_label(b)} {round(b['gain'] * 100)}%" for b in known)
  gains = [b["gain"] for b in known]
  if max(gains) - min(gains) > BAND_GAIN_SPREAD:
    takeaways.append(f"{what} changes with speed ({parts}). The tune is right at one speed and off at another; "
                     "the speed breakpoints are the place to look.")
    return [f"By speed: {parts}. The response is not the same at every speed, which points at the speed breakpoints "
            "rather than a single gain."]
  return [f"By speed: {parts}. The response was consistent across speeds."]


def _band(rmse, good, fair):
  if rmse <= good:
    return "closely"
  if rmse <= fair:
    return "with some visible error"
  return "loosely"


def _lateral_findings(m, takeaways):
  """Plain-language summary and notes for steering. Numbers stay in the metrics; the words say what it felt like."""
  if m.get("status") != "ok":
    return (f"Not enough engaged steering above {LAT_MIN_SPEED * MPH:.0f} mph ({LAT_MIN_SPEED * 3.6:.0f} km/h) to judge "
            f"({_fmt_time(m.get('engaged_s', 0))} counted)."), []
  notes = [f"The car starts reacting about {m['lag_s']} s after openpilot asks. After allowing for that, it typically stayed "
           f"within {m['rmse']} m/s² of the planned path, {_feel(m['rmse'], LAT_RMSE_GOOD, LAT_RMSE_FAIR)}."]
  g = m.get("curve_gain")
  if g is not None:
    pct = round(g * 100)
    if g < GAIN_LOW:
      notes.append(f"In curves the car turned about {100 - pct}% less than asked, so it tends to run wide. If this repeats "
                   "across drives, the steering feedforward (or the steer ratio) is a little low.")
      takeaways.append(f"Under-turning in curves ({pct}% of the request): consider a slightly higher feedforward or check the steer ratio.")
    elif g > GAIN_HIGH:
      notes.append(f"In curves the car turned about {pct - 100}% more than asked, so it tends to cut in. If this repeats "
                   "across drives, the steering feedforward (or the steer ratio) is a little high.")
      takeaways.append(f"Over-turning in curves ({pct}% of the request): consider a slightly lower feedforward or check the steer ratio.")
    else:
      notes.append(f"In curves the car turned almost exactly as much as asked ({pct}%).")
  else:
    notes.append("There were no sustained curves, so curve response could not be measured this drive.")
  notes.extend(_band_notes(m.get("speed_bands"), "Curve response", takeaways))
  sat = m.get("saturated_curve_frac")
  if sat is not None and sat > SATURATION_NOTABLE:
    notes.append(f"Steering was at its limit for {round(sat * 100)}% of the time in curves, meaning the car could not turn as hard "
                 "as openpilot wanted. Expect to take over in the sharpest curves; the steering limit is a car-specific setting.")
    takeaways.append(f"Steering hit its limit {round(sat * 100)}% of curve time: the sharpest curves are beyond what the car will do on its own.")
  w = m.get("wobble_ratio")
  if w is not None:
    if w > WOBBLE_RATIO_HIGH:
      notes.append(f"The wheel made quick back-and-forth corrections about {w}× stronger than the road called for. That is the "
                   "wobble or ping-pong you can feel on straights; it usually means the gain is a little high or the friction/damping a little low.")
      takeaways.append(f"Steering wobble ({w}× the road's demand): try a little less gain or a little more friction/damping.")
    else:
      notes.append(f"Steering was steady, with no unnecessary back-and-forth (ratio {w}, where about 1 is ideal).")
  b = m.get("straight_bias")
  if b is not None and abs(b) > LAT_BIAS_NOTABLE:
    side = "left" if b > 0 else "right"
    notes.append(f"On straight roads the car sat slightly to the {side} of the planned line ({abs(b)} m/s² on average). "
                 "If it feels like a constant pull, a small steering-offset change fixes this, not the gains.")
    takeaways.append(f"Constant {side} lean on straights: adjust the steering offset slightly.")
  if m.get("steer_overrides"):
    notes.append(f"You took the wheel {m['steer_overrides']} time(s) while engaged.")
  quality = f"Steering followed the planned path {_band(m['rmse'], LAT_RMSE_GOOD, LAT_RMSE_FAIR)}"
  if w is not None and w > WOBBLE_RATIO_HIGH:
    quality += ", but with some wobble"
  return f"{quality} over {_fmt_time(m['engaged_s'])} of engaged driving.", notes


def _longitudinal_findings(m, takeaways):
  if m.get("status") != "ok":
    return f"Not enough engaged openpilot speed control to judge ({_fmt_time(m.get('engaged_s', 0))} counted).", []
  notes = [f"The car starts reacting about {m['lag_s']} s after openpilot asks. After allowing for that, it typically stayed "
           f"within {m['rmse']} m/s² of the planned acceleration, {_feel(m['rmse'], LONG_RMSE_GOOD, LONG_RMSE_FAIR)}."]
  g = m.get("gain")
  gain_off = g is not None and not (GAIN_LOW <= g <= GAIN_HIGH)
  if gain_off:
    pct = round(g * 100)
    if g < GAIN_LOW:
      notes.append(f"Overall the car delivered about {pct}% of the acceleration and braking openpilot asked for, so it feels lazier than planned.")
      takeaways.append(f"Speed control under-delivers ({pct}% of the request): the car does less than the planner asks.")
    else:
      notes.append(f"Overall the car delivered about {pct}% of the acceleration and braking openpilot asked for, so it feels sharper than planned.")
      takeaways.append(f"Speed control over-delivers ({pct}% of the request): the car does more than the planner asks.")
  notes.extend(_band_notes(m.get("speed_bands"), "Speed-control response", takeaways))
  bb = m.get("brake_bias")
  if bb is not None:
    if bb > LONG_BIAS_NOTABLE:
      notes.append(f"When braking, the car slowed {bb} m/s² less than planned on average, so it closes on traffic a little more than intended.")
      takeaways.append(f"Braking is lighter than planned by {bb} m/s²: the car arrives a bit hot behind a slowing lead.")
    elif bb < -LONG_BIAS_NOTABLE:
      notes.append(f"When braking, the car slowed {abs(bb)} m/s² more than planned on average, so braking can feel early or strong.")
      takeaways.append(f"Braking is stronger than planned by {abs(bb)} m/s²: stops may feel abrupt.")
    else:
      notes.append("Braking matched the plan.")
  ab = m.get("accel_bias")
  if ab is not None:
    if ab < -LONG_BIAS_NOTABLE:
      notes.append(f"When accelerating, the car pulled away {abs(ab)} m/s² less than planned on average (sluggish).")
      takeaways.append(f"Acceleration is weaker than planned by {abs(ab)} m/s²: pull-away feels sluggish.")
    elif ab > LONG_BIAS_NOTABLE:
      notes.append(f"When accelerating, the car pulled away {ab} m/s² more than planned on average (a surge).")
      takeaways.append(f"Acceleration is stronger than planned by {ab} m/s²: pull-away can surge.")
    else:
      notes.append("Acceleration matched the plan.")
  j = m.get("jerk_p95")
  if j is not None:
    if j > JERK_HIGH:
      notes.append(f"Speed changes were abrupt (peak jerk {j} m/s³, above {JERK_HIGH}). Look at the acceleration profile or the actuator delay.")
      takeaways.append(f"Abrupt speed changes (jerk {j} m/s³): smooth the acceleration profile or check the actuator delay.")
    else:
      notes.append(f"Speed changes were smooth (peak jerk {j} m/s³).")
  if m.get("hard_brakes"):
    notes.append(f"{m['hard_brakes']} hard braking event(s) (harder than {abs(HARD_BRAKE)} m/s²) while engaged; worth a look at those "
                 "moments in the charts.")
  if m.get("gas_overrides"):
    notes.append(f"You pressed the gas {m['gas_overrides']} time(s) while engaged, usually a sign the car felt slow to you.")
  quality = f"Speed control followed the plan {_band(m['rmse'], LONG_RMSE_GOOD, LONG_RMSE_FAIR)}"
  if gain_off:
    quality += f", but delivered {'less' if g < GAIN_LOW else 'more'} than asked"
  elif j is not None and j > JERK_HIGH:
    quality += ", but felt jerky"
  return f"{quality} over {_fmt_time(m['engaged_s'])} of engaged driving.", notes


def analyze(rows, min_engaged_s=20.0):
  c = _as_arrays(rows)
  t = c["t"]
  seg = _segments(t)
  dt = _dt(t)
  lat = _analyze_lateral(c, seg, dt, min_engaged_s)
  lon = _analyze_longitudinal(c, seg, dt, min_engaged_s)
  takeaways = []
  lat_summary, lat_notes = _lateral_findings(lat, takeaways)
  lon_summary, lon_notes = _longitudinal_findings(lon, takeaways)
  if not takeaways and (lat.get("status") == "ok" or lon.get("status") == "ok"):
    takeaways.append("Nothing stood out this drive. Small errors are normal; compare a few drives before changing the tune.")
  duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0
  return {
    "duration_s": round(duration, 1),
    "samples": int(len(t)),
    "sample_dt_s": round(dt, 3),
    "disengagements": _rising_edges(~(c["enabled"] > 0.5)) if len(t) else 0,
    "takeaways": takeaways[:4],
    "lateral": {**lat, "summary": lat_summary, "notes": lat_notes},
    "longitudinal": {**lon, "summary": lon_summary, "notes": lon_notes},
    "method": ("How this was scored: only moments when openpilot was steering (no hands on the wheel, above "
               f"{LAT_MIN_SPEED * MPH:.0f} mph) or controlling speed (no gas, not stopped) count. The car's response is shifted "
               "by its measured reaction delay before it is compared with the request. Steering 'measured' comes from the "
               "steering angle through the vehicle model, so a wrong steer ratio shows up as a curve-response error. "
               "These are heuristics from one drive, not a verdict on the tune."),
  }


def overview(rows, buckets=1200):
  """Bucket-averaged series for a whole-drive chart, plus the per-bucket worst error."""
  c = _as_arrays(rows)
  n = len(c["t"])
  if n == 0:
    return {"t": [], "series": {}}
  edges = np.linspace(0, n, min(buckets, n) + 1).astype(int)
  out = {k: [] for k in ("t", "v", "lat_des", "lat_act", "long_des", "long_act", "lat_active", "long_active",
                         "lat_err_max", "long_err_max")}
  t0 = c["t"][0]
  for s, e in zip(edges[:-1], edges[1:]):
    if e <= s:
      continue
    sl = slice(s, e)
    out["t"].append(round(float(np.mean(c["t"][sl]) - t0), 2))
    for k in ("v", "lat_des", "lat_act", "long_des", "long_act"):
      out[k].append(round(float(np.mean(c[k][sl])), 3))
    out["lat_active"].append(round(float(np.mean(c["lat_active"][sl])), 2))
    out["long_active"].append(round(float(np.mean(c["long_active"][sl])), 2))
    out["lat_err_max"].append(round(float(np.max(np.abs(c["lat_act"][sl] - c["lat_des"][sl]))), 3))
    out["long_err_max"].append(round(float(np.max(np.abs(c["long_act"][sl] - c["long_des"][sl]))), 3))
  return out


def window(rows, start_s, end_s, max_points=3000):
  """Full-resolution rows between start_s and end_s (seconds from the drive's first sample)."""
  data = np.asarray(rows, dtype=float).reshape(-1, len(COLUMNS))
  if not len(data):
    return []
  rel = data[:, 0] - data[0, 0]
  sel = data[(rel >= start_s) & (rel <= end_s)]
  stride = max(1, int(np.ceil(len(sel) / max_points)))
  sel = sel[::stride].copy()
  sel[:, 0] -= data[0, 0]
  return np.round(sel, 4).tolist()


# =====================================================================================================
# Sampling
# =====================================================================================================

def _f(x, default=0.0):
  try:
    v = float(x)
    return v if np.isfinite(v) else default
  except Exception:
    return default


def build_row(sm):
  cs = sm["controlsState"]
  cc = sm["carControl"]
  car_state = sm["carState"]
  plan = sm["longitudinalPlan"]
  v = max(0.0, _f(getattr(car_state, "vEgo", 0.0)))
  have_cc = sm.recv_frame["carControl"] > 0
  lat_p = lat_i = lat_d = lat_f = 0.0
  lat_sat = 0
  try:
    lcs = cs.lateralControlState
    which = lcs.which()
    st = getattr(lcs, which)
    lat_sat = int(bool(getattr(st, "saturated", False)))
    if which in ("pidState", "torqueState"):
      lat_p, lat_i, lat_f = _f(st.p), _f(st.i), _f(st.f)
      lat_d = _f(getattr(st, "d", 0.0)) if which == "torqueState" else 0.0
  except Exception:
    pass
  try:
    long_state = LONG_STATES.get(str(cs.longControlState), 0)
  except Exception:
    long_state = 0
  t = sm.logMonoTime["longitudinalPlan"] / 1e9 if sm.logMonoTime["longitudinalPlan"] else time.monotonic()
  return [
    round(t, 3), round(v, 3), round(_f(getattr(car_state, "aEgo", 0.0)), 3),
    int(have_cc and bool(cc.enabled)), int(have_cc and bool(cc.latActive)), int(have_cc and bool(cc.longActive)),
    int(bool(getattr(car_state, "steeringPressed", False))), int(bool(getattr(car_state, "gasPressed", False))),
    int(bool(getattr(car_state, "brakePressed", False))),
    round(_f(cs.desiredCurvature) * v * v, 4), round(_f(cs.curvature) * v * v, 4),
    round(_f(getattr(plan, "aTarget", 0.0)), 4), round(_f(getattr(car_state, "aEgo", 0.0)), 4), long_state,
    round(lat_p, 4), round(lat_i, 4), round(lat_d, 4), round(lat_f, 4),
    round(_f(cs.upAccelCmd), 4), round(_f(cs.uiAccelCmd), 4), round(_f(cs.ufAccelCmd), 4), lat_sat,
  ]


def _fmt(x):
  # Numpy scalars repr as "np.float64(...)", which read_rows cannot parse back.
  if isinstance(x, (bool, np.bool_, int, np.integer)):
    return str(int(x))
  return repr(float(x))


class DrivePlots:
  SERVICES = ["controlsState", "carControl", "carState", "longitudinalPlan"]

  def __init__(self, root, is_onroad, submaster_factory=None, clock=time.monotonic):
    self.root = Path(root)
    self.is_onroad = is_onroad
    self.clock = clock
    self._submaster_factory = submaster_factory
    self.lock = threading.Lock()
    self.thread = None
    self.last_client = -1e9
    self.buffer = deque(maxlen=int(LIVE_BUFFER_S / NOMINAL_DT))
    self.seq = 0
    self.last_sample_wall = 0.0
    self.last_error = ""
    self.rec = None            # dict while recording
    self._live_cache = (-1, None)

  # ---------------- lifecycle ----------------
  def _make_submaster(self):
    if self._submaster_factory is not None:
      return self._submaster_factory(self.SERVICES)
    from cereal import messaging
    return messaging.SubMaster(self.SERVICES, poll="longitudinalPlan")

  def touch(self):
    with self.lock:
      self.last_client = self.clock()
      self._ensure_thread_locked()

  def _ensure_thread_locked(self):
    if self.thread is not None and self.thread.is_alive():
      return
    self.thread = threading.Thread(target=self._run, name="galaxy-drive-plots", daemon=True)
    self.thread.start()

  def _should_run(self):
    with self.lock:
      return self.rec is not None or (self.clock() - self.last_client) < CLIENT_IDLE_TIMEOUT_S

  def _run(self):
    try:
      sm = self._make_submaster()
    except Exception as e:
      with self.lock:
        self.last_error = f"subscribe failed: {e}"
        self.thread = None
      return
    last_onroad_check = -1e9
    while self._should_run():
      try:
        self.step(sm)
        now = self.clock()
        if now - last_onroad_check >= 1.0:
          last_onroad_check = now
          self._check_autostop(now)
      except Exception as e:
        with self.lock:
          self.last_error = str(e)
        time.sleep(0.1)
    with self.lock:
      self.thread = None

  def step(self, sm):
    sm.update(100)
    if not sm.updated["longitudinalPlan"]:
      return False
    row = build_row(sm)
    with self.lock:
      self.seq += 1
      self.buffer.append((self.seq, row))
      self.last_sample_wall = time.time()
      self.last_error = ""
      rec = self.rec
      if rec is not None:
        rec["writer"].writerow([_fmt(x) for x in row])
        rec["rows"] += 1
        if rec["first_t"] is None:
          rec["first_t"] = row[0]
        rec["last_t"] = row[0]
        if rec["rows"] % FLUSH_EVERY_ROWS == 0:
          rec["file"].flush()
    return True

  def _check_autostop(self, now):
    with self.lock:
      rec = self.rec
    if rec is None:
      return
    onroad = bool(self.is_onroad())
    reason = None
    with self.lock:
      if onroad:
        rec["seen_onroad"] = True
        rec["offroad_since"] = None
      elif rec["seen_onroad"]:
        if rec["offroad_since"] is None:
          rec["offroad_since"] = now
        elif now - rec["offroad_since"] >= OFFROAD_AUTOSTOP_S:
          reason = "drive ended"
      if reason is None and now - rec["started_mono"] >= MAX_RECORDING_S:
        reason = "time limit"
    if reason:
      self.stop_recording(reason=reason)

  # ---------------- recording ----------------
  def _session_dir(self, session_id):
    session_id = str(session_id)
    if not session_id.isalnum():
      raise ValueError("bad session id")
    return self.root / session_id

  def start_recording(self, meta=None):
    with self.lock:
      if self.rec is not None:
        return self._rec_status_locked()
      self.root.mkdir(parents=True, exist_ok=True)
      session_id = time.strftime("%Y%m%d%H%M%S", time.localtime())
      d = self.root / session_id
      suffix = 0
      while d.exists():
        suffix += 1
        d = self.root / f"{session_id}{suffix}"
      d.mkdir(parents=True)
      m = dict(meta or {})
      m.update({"id": d.name, "started_at": time.time(), "status": "recording", "columns": COLUMNS})
      (d / "meta.json").write_text(json.dumps(m, indent=1))
      f = open(d / "samples.csv", "w", newline="")
      w = csv.writer(f)
      w.writerow(COLUMNS)
      self.rec = {"id": d.name, "dir": d, "file": f, "writer": w, "rows": 0, "first_t": None, "last_t": None,
                  "started_mono": self.clock(), "seen_onroad": bool(self.is_onroad()), "offroad_since": None}
      self._ensure_thread_locked()
      return self._rec_status_locked()

  def stop_recording(self, reason="stopped", background=True):
    with self.lock:
      rec = self.rec
      self.rec = None
    if rec is None:
      return None
    try:
      rec["file"].flush()
      rec["file"].close()
    except Exception:
      pass
    if rec["rows"] == 0:
      # Nothing arrived (openpilot was not running); an empty session would only clutter the list.
      shutil.rmtree(rec["dir"], ignore_errors=True)
      return {"id": rec["id"], "rows": 0, "reason": reason, "discarded": True}
    self._update_meta(rec["dir"], {"status": "analyzing", "stopped_at": time.time(), "stop_reason": reason})
    if background:
      threading.Thread(target=self.finalize, args=(rec["dir"],), daemon=True).start()
    else:
      self.finalize(rec["dir"])
    return {"id": rec["id"], "rows": rec["rows"], "reason": reason}

  def _update_meta(self, d, updates):
    p = d / "meta.json"
    try:
      m = json.loads(p.read_text())
    except Exception:
      m = {"id": d.name}
    m.update(updates)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(m, indent=1))
    os.replace(tmp, p)
    return m

  def finalize(self, d):
    d = Path(d)
    try:
      rows = read_rows(d)
      result = analyze(rows)
      result["overview"] = overview(rows)
      tmp = d / "analysis.tmp"
      tmp.write_text(json.dumps(result))
      os.replace(tmp, d / "analysis.json")
      raw = d / "samples.csv"
      if raw.exists():
        with open(raw, "rb") as src, gzip.open(d / "samples.csv.gz.tmp", "wb") as dst:
          shutil.copyfileobj(src, dst)
        os.replace(d / "samples.csv.gz.tmp", d / "samples.csv.gz")
        raw.unlink()
      self._update_meta(d, {"status": "done", "duration_s": result["duration_s"],
                            "lateral_summary": result["lateral"]["summary"],
                            "longitudinal_summary": result["longitudinal"]["summary"]})
    except Exception as e:
      self._update_meta(d, {"status": "error", "error": str(e)})

  def recover_interrupted(self):
    """A session left 'recording'/'analyzing' by a Galaxy restart is finalized with what was written."""
    if not self.root.exists():
      return
    with self.lock:
      active = self.rec["id"] if self.rec else None
    for d in self.root.iterdir():
      if not d.is_dir() or d.name == active:
        continue
      try:
        m = json.loads((d / "meta.json").read_text())
      except Exception:
        continue
      if m.get("status") in ("recording", "analyzing") and not (d / "analysis.json").exists():
        self._update_meta(d, {"status": "analyzing", "stop_reason": "interrupted (Galaxy restarted)"})
        self.finalize(d)

  # ---------------- queries ----------------
  def _rec_status_locked(self):
    if self.rec is None:
      return None
    r = self.rec
    elapsed = (r["last_t"] - r["first_t"]) if r["first_t"] is not None else 0.0
    return {"id": r["id"], "rows": r["rows"], "elapsed_s": round(elapsed, 1)}

  def live(self, since=0):
    self.touch()
    with self.lock:
      rows = [[s, *r] for s, r in self.buffer if s > since]
      seq = self.seq
      age = time.time() - self.last_sample_wall if self.last_sample_wall else None
      rec = self._rec_status_locked()
      err = self.last_error
      cache_seq, cache = self._live_cache
      window_rows = None
      if cache_seq != seq:
        last_t = self.buffer[-1][1][0] if self.buffer else 0.0
        window_rows = [r for _, r in self.buffer if r[0] >= last_t - LIVE_WINDOW_S]
    if window_rows is not None:
      cache = None
      if len(window_rows) > 1:
        a = analyze(window_rows, min_engaged_s=10.0)
        cache = {k: a[k] for k in ("lateral", "longitudinal")}
      with self.lock:
        self._live_cache = (seq, cache)
    return {
      "columns": ["seq", *COLUMNS],
      "rows": rows,
      "seq": seq,
      "sampleAgeSeconds": None if age is None else round(age, 3),
      "stale": age is None or age > 1.5,
      "recording": rec,
      "lastError": err,
      "liveAnalysis": cache,
      "liveWindowSeconds": LIVE_WINDOW_S,
    }

  def list_sessions(self):
    out = []
    if not self.root.exists():
      return out
    for d in sorted(self.root.iterdir(), reverse=True):
      if not d.is_dir():
        continue
      try:
        m = json.loads((d / "meta.json").read_text())
      except Exception:
        continue
      out.append({k: m.get(k) for k in ("id", "started_at", "stopped_at", "status", "stop_reason", "duration_s",
                                        "lateral_summary", "longitudinal_summary", "car", "error")})
    return out

  def get_session(self, session_id):
    d = self._session_dir(session_id)
    if not d.is_dir():
      return None
    m = json.loads((d / "meta.json").read_text())
    analysis = None
    if (d / "analysis.json").exists():
      analysis = json.loads((d / "analysis.json").read_text())
    return {"meta": m, "analysis": analysis}

  def get_window(self, session_id, start_s, end_s):
    d = self._session_dir(session_id)
    return {"columns": COLUMNS, "rows": window(read_rows(d), start_s, end_s)}

  def csv_path(self, session_id):
    d = self._session_dir(session_id)
    for name in ("samples.csv.gz", "samples.csv"):
      if (d / name).exists():
        return d / name
    return None

  def delete_session(self, session_id):
    d = self._session_dir(session_id)
    with self.lock:
      if self.rec is not None and self.rec["id"] == d.name:
        return False
    if not d.is_dir():
      return False
    shutil.rmtree(d)
    return True


def read_rows(d):
  """Read a session's samples, tolerating a torn last line from an interrupted write."""
  d = Path(d)
  if (d / "samples.csv.gz").exists():
    text = gzip.open(d / "samples.csv.gz", "rt").read()
  elif (d / "samples.csv").exists():
    text = (d / "samples.csv").read_text()
  else:
    return np.zeros((0, len(COLUMNS)))
  # Older sessions may lack columns added later; they are read by header name and missing ones are zero.
  reader = csv.reader(io.StringIO(text))
  header = next(reader, None)
  if not header or header[0] != "t":
    return np.zeros((0, len(COLUMNS)))
  order = [header.index(name) if name in header else None for name in COLUMNS]
  rows = []
  for rec in reader:
    if len(rec) != len(header):
      continue
    try:
      rows.append([0.0 if i is None else float(rec[i]) for i in order])
    except ValueError:
      continue
  return np.asarray(rows, dtype=float).reshape(-1, len(COLUMNS))
