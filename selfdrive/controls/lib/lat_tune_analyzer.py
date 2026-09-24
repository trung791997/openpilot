"""Offline adaptive P trim for modified-EPS Honda lateral PID (item 116 rules, FLM-style trials).

Pure module: no Params, no Flask. Callers are the Galaxy workspace, the CLI and tests.
Frames come from rlogs (frames_from_log). One trial = one step per knot, bounded 0.85..1.15.
"""
import json
import math
import zlib

MPH_TO_MS = 0.44704
KNOTS_MPH = (20.0, 30.0, 40.0, 50.0)
KNOTS = tuple(k * MPH_TO_MS for k in KNOTS_MPH)

STEP = 0.05
FACTOR_MIN = 0.85
FACTOR_MAX = 1.15
MAX_NEIGHBOUR_GAP = 0.10
MIN_MINUTES = 3.0
MIN_SPEED = 4.0
STRAIGHT_DEG = 3.0
CURVE_DEG = 5.0
MIN_CURVE_S = 30.0

SIGN_RATE_MAX = 1.0        # /s, straight error sign changes
SIGN_RATE_UP_MAX = 0.8     # /s, no step up above this
CURVE_RATIO_LOW = 0.95
CURVE_RATIO_HIGH = 1.03
PRESS_RATE_UP_MAX = 1.5    # override onsets per engaged minute; no step up above this
OSC_GROWTH = 1.15
PRESS_GROWTH = 1.25
PRESS_SLACK = 0.2

STATE_VERSION = 1
APPLY_MODE = 2

# Manual lateral gains the factor is relative to. A change to any of them resets the learned state.
TUNING_KEYS = (
  "LatPScaleLowSpeed", "LatPScaleStandard", "LatPScaleHighway",
  "LatIScaleLowSpeed", "LatIScaleStandard", "LatIScaleHighway",
  "LatFScaleLowSpeed", "LatFScaleStandard", "LatFScaleHighway", "LatGainSchedule",
  "HondaLateralPidKpScale", "HondaLateralPidKiScale",
  "HondaCenterScale", "HondaCenterBoostThreshold", "HondaCenterBoostMinSpeed",
  "HondaTorqueLowPassFilter", "HondaLpfTauLowSpeed", "HondaLpfTauStandard", "HondaLpfTauHighway",
  "HondaOverrideTorqueScale", "HondaOverrideFadeUpSecs", "HondaOverrideFadeDownSecs",
  # Selects the rack map that turns curvature into the target wheel angle (firmware VGR table vs
  # the road-measured curve). Switching moves the centre gain ~10 % and the taper, so every angle
  # the factor was learned against changes. The paramsd-learned ratio is deliberately not here:
  # it drifts continuously, and resetting on it would mean never learning.
  "NrdrLatUseFirmwareVgr",
)

_FIELDS = ("n", "n_act", "n_st", "e2_st", "sc", "n_cur", "ang_cur", "des_cur", "press")


def knot_weights(v_ego):
  """(index, weight) pairs of the two knots bracketing v_ego; flat past the ends."""
  if v_ego <= KNOTS[0]:
    return ((0, 1.0),)
  for i in range(1, len(KNOTS)):
    if v_ego <= KNOTS[i]:
      t = (v_ego - KNOTS[i - 1]) / (KNOTS[i] - KNOTS[i - 1])
      return ((i - 1, 1.0 - t), (i, t))
  return ((len(KNOTS) - 1, 1.0),)


def interp_factor(factors, v_ego):
  return sum(factors[i] * w for i, w in knot_weights(v_ego))


def tuning_fingerprint(values):
  """Short hash of the manual tuning. `values` maps key -> raw param value (str/bytes/None)."""
  parts = []
  for k in TUNING_KEYS:
    v = values.get(k)
    if isinstance(v, bytes):
      v = v.decode("utf-8", "replace")
    parts.append(f"{k}={'' if v is None else str(v).strip()}")
  return f"{zlib.crc32(';'.join(parts).encode()):08x}"


def default_state(tuning=None):
  return {"version": STATE_VERSION, "knots_mph": list(KNOTS_MPH), "factor": [1.0] * len(KNOTS),
          "prev": [None] * len(KNOTS), "carry": [None] * len(KNOTS), "drives": 0, "last": [], "tuning": tuning}


def parse_state(raw):
  try:
    if isinstance(raw, bytes):
      raw = raw.decode("utf-8")
    s = json.loads(raw) if isinstance(raw, str) else raw
    if not isinstance(s, dict) or s.get("version") != STATE_VERSION or list(s.get("knots_mph", ())) != list(KNOTS_MPH):
      return default_state()
    f = [float(x) for x in s["factor"]]
    if len(f) != len(KNOTS) or not all(math.isfinite(x) and FACTOR_MIN - 1e-9 <= x <= FACTOR_MAX + 1e-9 for x in f):
      return default_state()
    prev = s.get("prev") or [None] * len(KNOTS)
    if len(prev) != len(KNOTS):
      prev = [None] * len(KNOTS)
    carry = s.get("carry") or [None] * len(KNOTS)
    if len(carry) != len(KNOTS):
      carry = [None] * len(KNOTS)
    carry = [_clean_acc(c) for c in carry]
    tuning = s.get("tuning")
    return {"version": STATE_VERSION, "knots_mph": list(KNOTS_MPH), "factor": f, "prev": prev, "carry": carry,
            "drives": int(s.get("drives", 0)), "last": list(s.get("last", [])),
            "tuning": tuning if isinstance(tuning, str) else None}
  except (TypeError, ValueError, KeyError, AttributeError, UnicodeDecodeError):
    return default_state()


def _clean_acc(c):
  if not isinstance(c, dict):
    return None
  try:
    out = {k: float(c.get(k, 0.0)) for k in _FIELDS}
  except (TypeError, ValueError):
    return None
  return out if all(math.isfinite(v) and v >= 0 for v in out.values()) else None


class DriveStats:
  """Per-knot accumulators for one drive. Plain floats so observe() stays cheap at 100 Hz."""
  def __init__(self, carry=None, tuning=None):
    self.tuning = tuning
    self.acc = [dict.fromkeys(_FIELDS, 0.0) for _ in KNOTS]
    for a, c in zip(self.acc, carry or [None] * len(KNOTS), strict=True):
      if c is not None:
        a.update(c)
    self.prev_sign = 0.0
    self.prev_pressed = False

  def observe(self, v_ego, desired_deg, angle_deg, pressed, lane_change, steer_limited):
    """Call once per engaged frame."""
    onset = pressed and not self.prev_pressed
    self.prev_pressed = pressed
    if v_ego < MIN_SPEED:
      self.prev_sign = 0.0
      return
    ws = knot_weights(v_ego)
    for i, w in ws:
      a = self.acc[i]
      a["n_act"] += w
      if onset:
        a["press"] += w
    if pressed or lane_change or steer_limited:
      self.prev_sign = 0.0
      return
    err = desired_deg - angle_deg
    straight = abs(desired_deg) < STRAIGHT_DEG
    curve = abs(desired_deg) > CURVE_DEG
    sign = (1.0 if err > 0.0 else -1.0 if err < 0.0 else 0.0) if straight else 0.0
    changed = straight and self.prev_sign != 0.0 and sign != 0.0 and sign != self.prev_sign
    for i, w in ws:
      a = self.acc[i]
      a["n"] += w
      if straight:
        a["n_st"] += w
        a["e2_st"] += w * err * err
        if changed:
          a["sc"] += w
      elif curve:
        a["n_cur"] += w
        s = 1.0 if desired_deg > 0.0 else -1.0
        a["ang_cur"] += w * angle_deg * s
        a["des_cur"] += w * abs(desired_deg)
    if sign != 0.0:
      self.prev_sign = sign
    elif not straight:
      self.prev_sign = 0.0

  def has_data(self):
    return any(a["n"] > 0 for a in self.acc)

  def to_json(self):
    return json.dumps({"version": STATE_VERSION, "knots_mph": list(KNOTS_MPH), "tuning": self.tuning,
                       "acc": [{k: round(v, 4) for k, v in a.items()} for a in self.acc]}, separators=(",", ":"))

  @staticmethod
  def from_json(raw):
    try:
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
      d = json.loads(raw) if isinstance(raw, str) else raw
      if not isinstance(d, dict) or d.get("version") != STATE_VERSION or list(d.get("knots_mph", ())) != list(KNOTS_MPH):
        return None
      tuning = d.get("tuning")
      st = DriveStats(tuning=tuning if isinstance(tuning, str) else None)
      for a, src in zip(st.acc, d["acc"], strict=True):
        for k in _FIELDS:
          v = float(src.get(k, 0.0))
          if not math.isfinite(v) or v < 0:
            return None
          a[k] = v
      return st
    except (TypeError, ValueError, KeyError, AttributeError, UnicodeDecodeError):
      return None


def knot_metrics(a, dt=0.01):
  minutes = a["n"] * dt / 60.0
  act_min = a["n_act"] * dt / 60.0
  return {
    "min": minutes,
    "straight_rms": math.sqrt(a["e2_st"] / a["n_st"]) if a["n_st"] > 0 else None,
    "sign_rate": a["sc"] / (a["n_st"] * dt) if a["n_st"] * dt >= 10.0 else None,
    "curve_ratio": a["ang_cur"] / a["des_cur"] if a["n_cur"] * dt >= MIN_CURVE_S and a["des_cur"] > 0 else None,
    "press_rate": a["press"] / act_min if act_min > 0 else None,
  }


def update_state(state, stats, mode):
  """One drive's step. Returns the new state; `last` lists what each knot did and why."""
  state = parse_state(state) if not isinstance(state, dict) else parse_state(json.dumps(state))
  f = list(state["factor"])
  prev = list(state["prev"])
  carry = [None] * len(KNOTS)
  last = []
  for i, a in enumerate(stats.acc):
    m = knot_metrics(a)
    k = f"{KNOTS_MPH[i]:g}mph"
    if m["min"] < MIN_MINUTES or m["sign_rate"] is None:
      # Too little data: nothing moves, and the data carries into the next drive (same factor).
      carry[i] = {kk: round(v, 4) for kk, v in a.items()}
      last.append(f"{k}: hold ({m['min']:.1f} min of data, carried)")
      continue
    p = prev[i]
    step = 0.0
    why = "hold"
    stepped_up = p is not None and p.get("stepped", 0.0) > 0 and mode == APPLY_MODE
    press_now = m["press_rate"] or 0.0
    if stepped_up and p.get("sign_rate") and m["sign_rate"] > p["sign_rate"] * OSC_GROWTH:
      step, why = -STEP, f"revert: sign changes {p['sign_rate']:.2f} -> {m['sign_rate']:.2f}/s after the last increase"
    elif stepped_up and p.get("press_rate") is not None and press_now > p["press_rate"] * PRESS_GROWTH + PRESS_SLACK:
      step, why = -STEP, f"revert: override onsets {p['press_rate']:.2f} -> {press_now:.2f}/min after the last increase"
    elif m["sign_rate"] > SIGN_RATE_MAX:
      step, why = -STEP, f"down: sign changes {m['sign_rate']:.2f}/s > {SIGN_RATE_MAX}"
    elif m["curve_ratio"] is not None and m["curve_ratio"] > CURVE_RATIO_HIGH:
      step, why = -STEP, f"down: curve ratio {m['curve_ratio']:.3f} > {CURVE_RATIO_HIGH}"
    elif (m["curve_ratio"] is not None and m["curve_ratio"] < CURVE_RATIO_LOW and m["sign_rate"] < SIGN_RATE_UP_MAX
          and press_now < PRESS_RATE_UP_MAX):
      step, why = STEP, f"up: curve ratio {m['curve_ratio']:.3f} < {CURVE_RATIO_LOW}"
    elif m["curve_ratio"] is not None and m["curve_ratio"] < CURVE_RATIO_LOW:
      why = f"hold: curve ratio {m['curve_ratio']:.3f} low but sign changes {m['sign_rate']:.2f}/s or onsets {press_now:.2f}/min too high"
    new = min(max(f[i] + step, FACTOR_MIN), FACTOR_MAX)
    prev[i] = {"factor": f[i], "stepped": new - f[i], "sign_rate": m["sign_rate"], "curve_ratio": m["curve_ratio"],
               "straight_rms": m["straight_rms"], "press_rate": m["press_rate"], "min": round(m["min"], 2)}
    f[i] = new
    last.append(f"{k}: {why} -> {new:.2f}")
  # neighbour smoothness: pull toward 1.0 whichever of a pair sits further from it
  for _ in range(len(f)):
    for i in range(len(f) - 1):
      if abs(f[i] - f[i + 1]) > MAX_NEIGHBOUR_GAP + 1e-9:
        j = i if abs(f[i] - 1.0) > abs(f[i + 1] - 1.0) else i + 1
        o = i + 1 if j == i else i
        f[j] = f[o] + math.copysign(MAX_NEIGHBOUR_GAP, f[j] - f[o])
  state.update({"factor": [round(x, 4) for x in f], "prev": prev, "carry": carry, "drives": state["drives"] + 1, "last": last})
  return state


# ---------------------------------------------------------------------------
# Trial building (offline). Percent values follow LatGainSchedule / Lat*Scale conventions.

SCHEMA_VERSION = 1
BAND_KEYS = ("LatPScaleLowSpeed", "LatPScaleStandard", "LatPScaleHighway")


def _param_str(v):
  if isinstance(v, bytes):
    v = v.decode("utf-8", "replace")
  return "" if v is None else str(v)


def effective_p_pct(raw_params):
  """P scale in percent at each knot, from the Lat*Scale bands and LatGainSchedule as LatControlPID reads them.

  raw_params maps key -> str/bytes/None (initData entries or Params.get values)."""
  from openpilot.selfdrive.controls.lib.latcontrol_pid import (
    _lat_pid_scale_banded, lat_gain_schedule_scale, parse_lat_gain_schedule)
  bands = []
  for key in BAND_KEYS:
    try:
      bands.append(min(max(float(_param_str(raw_params.get(key)) or 100.0) / 100.0, 0.0), 5.0))
    except ValueError:
      bands.append(1.0)
  try:
    sched = parse_lat_gain_schedule(_param_str(raw_params.get("LatGainSchedule")) or None)
  except Exception:
    sched = None
  out = []
  for v in KNOTS:
    banded = _lat_pid_scale_banded(v, *bands)
    out.append(round(lat_gain_schedule_scale(sched, "p", v, banded) * 100.0, 1))
  return out


def build_trial(stats, baseline, route_names, per_route, warnings):
  """One trial from one DriveStats over the selected routes. baseline = {"fingerprint", "pPct", "raw"}."""
  state = update_state(default_state(baseline.get("fingerprint")), stats, APPLY_MODE)
  knots = []
  for i, mph in enumerate(KNOTS_MPH):
    m = knot_metrics(stats.acc[i]) if stats.acc[i]["n"] > 0 else {"min": 0.0, "straight_rms": None, "sign_rate": None,
                                                                   "curve_ratio": None, "press_rate": None}
    reason = state["last"][i] if i < len(state["last"]) else ""
    decision = reason.split(":", 2)[1].strip().split(" ")[0] if reason.count(":") >= 1 else "hold"
    knots.append({"mph": mph, "minutes": round(m["min"], 2), "ready": m["min"] >= MIN_MINUTES,
                  "signRate": m["sign_rate"], "curveRatio": m["curve_ratio"], "straightRms": m["straight_rms"],
                  "pressRate": m["press_rate"], "factor": state["factor"][i], "decision": decision, "reason": reason})
  base_p = [float(x) for x in baseline["pPct"]]
  proposed = [round(base_p[i] * state["factor"][i], 1) for i in range(len(KNOTS))]
  return {"schemaVersion": SCHEMA_VERSION, "knotsMph": list(KNOTS_MPH), "routeNames": list(route_names),
          "warnings": list(warnings), "baseline": {"fingerprint": baseline.get("fingerprint"), "pPct": base_p,
                                                   "raw": {k: _param_str(v) for k, v in baseline.get("raw", {}).items()}},
          "knots": knots, "factors": list(state["factor"]), "proposedPPct": proposed, "perRoute": list(per_route),
          "applied": None}


def _interp_at_knots(v_mph, vals):
  out = []
  for k in KNOTS_MPH:
    if k <= v_mph[0]:
      out.append(vals[0])
    elif k >= v_mph[-1]:
      out.append(vals[-1])
    else:
      j = next(j for j in range(1, len(v_mph)) if v_mph[j] >= k)
      t = (k - v_mph[j - 1]) / (v_mph[j] - v_mph[j - 1])
      out.append(vals[j - 1] + t * (vals[j] - vals[j - 1]))
  return [round(x, 1) for x in out]


def build_schedule(trial, current_raw):
  """LatGainSchedule JSON for a trial. i/f are carried from current_raw (the schedule on the device now)."""
  from openpilot.selfdrive.controls.lib.latcontrol_pid import LAT_GAIN_SCHEDULE_LIMITS
  lo, hi = LAT_GAIN_SCHEDULE_LIMITS["p"]
  out = {"v_mph": list(KNOTS_MPH), "p": [round(min(max(x, lo), hi), 1) for x in trial["proposedPPct"]]}
  cur = None
  try:
    cur = json.loads(_param_str(current_raw)) if _param_str(current_raw).strip() else None
  except ValueError:
    cur = None
  if isinstance(cur, dict) and isinstance(cur.get("v_mph"), list) and len(cur["v_mph"]) >= 2:
    v_mph = [float(x) for x in cur["v_mph"]]
    for term in ("i", "f"):
      vals = cur.get(term)
      if isinstance(vals, list) and len(vals) == len(v_mph):
        t_lo, t_hi = LAT_GAIN_SCHEDULE_LIMITS[term]
        out[term] = [min(max(x, t_lo), t_hi) for x in _interp_at_knots(v_mph, [float(x) for x in vals])]
  return json.dumps(out, separators=(",", ":"))


# ---------------------------------------------------------------------------
# rlog -> frames. Only what DriveStats.observe needs; everything else is skipped.

from dataclasses import dataclass


@dataclass(slots=True)
class RouteLog:
  route: str
  segment: str
  log_path: str
  log_reader: object = None   # callable(path, **kw) -> iterable of messages; None = LogReader


def _default_log_reader(path, **kw):
  from openpilot.tools.lib.logreader import LogReader
  return LogReader(path, **kw)


def _steer_of(actuators):
  for name in ("torque", "steer"):
    if hasattr(actuators, name):
      return float(getattr(actuators, name))
  return None


class FrameSource:
  """Iterates one segment log and yields DriveStats.observe() arguments at each pidState controlsState."""

  def __init__(self, log_path, log_reader=None, should_continue=None):
    self.log_path = log_path
    self.log_reader = log_reader or _default_log_reader
    self.should_continue = should_continue
    self.tuning = {}      # TUNING_KEYS + BAND_KEYS values from initData, str
    self.n = 0

  def frames(self):
    wanted = set(TUNING_KEYS) | set(BAND_KEYS)
    v_ego = angle = 0.0
    pressed = lane_change = False
    cc_steer = co_steer = None
    for msg in self.log_reader(self.log_path):
      if self.should_continue is not None and not self.should_continue():
        return
      which = msg.which()
      if which == "initData":
        params = getattr(msg.initData, "params", None)
        entries = getattr(params, "entries", None)
        if entries is not None:
          for e in entries:
            if e.key in wanted:
              self.tuning[e.key] = _param_str(e.value)
        elif isinstance(params, dict):
          self.tuning.update({k: _param_str(v) for k, v in params.items() if k in wanted})
      elif which == "carState":
        cs = msg.carState
        v_ego, angle, pressed = float(cs.vEgo), float(cs.steeringAngleDeg), bool(cs.steeringPressed)
      elif which == "modelV2":
        lane_change = str(getattr(getattr(msg.modelV2, "meta", None), "laneChangeState", "off")) != "off"
      elif which == "carControl":
        cc_steer = _steer_of(msg.carControl.actuators)
      elif which == "carOutput":
        co_steer = _steer_of(msg.carOutput.actuatorsOutput)
      elif which == "controlsState":
        lcs = msg.controlsState.lateralControlState
        if lcs.which() != "pidState" or not lcs.pidState.active:
          continue
        steer_limited = cc_steer is not None and co_steer is not None and abs(cc_steer - co_steer) > 0.01
        self.n += 1
        yield v_ego, float(lcs.pidState.steeringAngleDesiredDeg), angle, pressed, lane_change, steer_limited


def analyze_sources(sources, should_continue=None, on_progress=None):
  """sources: RouteLog list in analysis order (oldest first). Returns a trial dict (build_trial)."""
  stats = DriveStats()
  per_route = {}
  warnings = []
  tuning_by_route = {}
  for idx, src in enumerate(sources):
    if on_progress is not None:
      on_progress(idx, len(sources), src)
    fs = FrameSource(src.log_path, log_reader=src.log_reader, should_continue=should_continue)
    route_stats = DriveStats()
    for frame in fs.frames():
      stats.observe(*frame)
      route_stats.observe(*frame)
    if fs.n == 0:
      warnings.append(f"{src.route}--{src.segment}: no engaged pidState frames (not modified-EPS PID, or never engaged)")
    if fs.tuning:
      tuning_by_route[src.route] = fs.tuning
    per_route.setdefault(src.route, [0.0] * len(KNOTS))
    for i in range(len(KNOTS)):
      per_route[src.route][i] = round(per_route[src.route][i] + knot_metrics(route_stats.acc[i])["min"], 2)
  route_names = list(dict.fromkeys(s.route for s in sources))
  latest = next((tuning_by_route[r] for r in reversed(route_names) if r in tuning_by_route), {})
  fps = {r: tuning_fingerprint(t) for r, t in tuning_by_route.items()}
  if len(set(fps.values())) > 1:
    warnings.append("routes were driven with different lateral tuning (fingerprint mismatch); the newest route's tuning is the baseline")
  baseline = {"fingerprint": tuning_fingerprint(latest) if latest else None, "pPct": effective_p_pct(latest), "raw": latest}
  return build_trial(stats, baseline, route_names, [{"route": r, "minutes": m} for r, m in per_route.items()], warnings)
