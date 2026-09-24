"""Offline adaptive P trim for modified-EPS Honda lateral PID (item 116 rules, FLM-style trials).

Pure module: no Params, no Flask. Callers are the Galaxy workspace, the CLI and tests.
Frames come from rlogs (frames_from_log). One trial = one step per speed band, bounded 0.85..1.15.
The bands are StarPilot's PID bands (LatControlPID._lat_pid_scale_banded): hard steps at 25 and 50 mph,
each with its own LatPScale*/LatIScale*/LatFScale* percent. Only P is proposed (item 116); I/F are carried.
"""
import json
import math
import zlib

MPH_TO_MS = 0.44704
# (name, low mph, high mph); v < 25 mph -> LowSpeed, v < 50 mph -> Standard, else Highway.
BANDS = (("LowSpeed", 0.0, 25.0), ("Standard", 25.0, 50.0), ("Highway", 50.0, None))
BAND_NAMES = tuple(b[0] for b in BANDS)
_BAND_EDGES = tuple(b[2] * MPH_TO_MS for b in BANDS[:-1])
PCT_STEP = 5          # Galaxy step for Lat*Scale*
PCT_MIN, PCT_MAX = 0, 500

STEP = 0.05          # factor grid; the smallest step
MAX_STEP = 0.15      # largest step per trial (owner, 2026-09-24: "up to 15 % per trial")
FACTOR_MIN = 0.85
FACTOR_MAX = 1.15
MAX_NEIGHBOUR_GAP = 0.15
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

STATE_VERSION = 2
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


def band_index(v_ego):
  """Band of v_ego, with the same strict < edges as _lat_pid_scale_banded."""
  for i, edge in enumerate(_BAND_EDGES):
    if v_ego < edge:
      return i
  return len(BANDS) - 1


def band_weights(v_ego):
  return ((band_index(v_ego), 1.0),)


def band_label(i):
  name, lo, hi = BANDS[i]
  return f"{name} {lo:g}-{hi:g} mph" if hi is not None else f"{name} {lo:g}+ mph"


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
  return {"version": STATE_VERSION, "bands": list(BAND_NAMES), "factor": [1.0] * len(BANDS),
          "prev": [None] * len(BANDS), "carry": [None] * len(BANDS), "drives": 0, "last": [], "tuning": tuning}


def parse_state(raw):
  try:
    if isinstance(raw, bytes):
      raw = raw.decode("utf-8")
    s = json.loads(raw) if isinstance(raw, str) else raw
    if not isinstance(s, dict) or s.get("version") != STATE_VERSION or list(s.get("bands", ())) != list(BAND_NAMES):
      return default_state()
    f = [float(x) for x in s["factor"]]
    if len(f) != len(BANDS) or not all(math.isfinite(x) and FACTOR_MIN - 1e-9 <= x <= FACTOR_MAX + 1e-9 for x in f):
      return default_state()
    prev = s.get("prev") or [None] * len(BANDS)
    if len(prev) != len(BANDS):
      prev = [None] * len(BANDS)
    carry = s.get("carry") or [None] * len(BANDS)
    if len(carry) != len(BANDS):
      carry = [None] * len(BANDS)
    carry = [_clean_acc(c) for c in carry]
    tuning = s.get("tuning")
    return {"version": STATE_VERSION, "bands": list(BAND_NAMES), "factor": f, "prev": prev, "carry": carry,
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
  """Per-band accumulators for one drive. Plain floats so observe() stays cheap at 100 Hz."""
  def __init__(self, carry=None, tuning=None):
    self.tuning = tuning
    self.acc = [dict.fromkeys(_FIELDS, 0.0) for _ in BANDS]
    for a, c in zip(self.acc, carry or [None] * len(BANDS), strict=True):
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
    ws = band_weights(v_ego)
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
    return json.dumps({"version": STATE_VERSION, "bands": list(BAND_NAMES), "tuning": self.tuning,
                       "acc": [{k: round(v, 4) for k, v in a.items()} for a in self.acc]}, separators=(",", ":"))

  @staticmethod
  def from_json(raw):
    try:
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
      d = json.loads(raw) if isinstance(raw, str) else raw
      if not isinstance(d, dict) or d.get("version") != STATE_VERSION or list(d.get("bands", ())) != list(BAND_NAMES):
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


def band_metrics(a, dt=0.01):
  minutes = a["n"] * dt / 60.0
  act_min = a["n_act"] * dt / 60.0
  return {
    "min": minutes,
    "straight_rms": math.sqrt(a["e2_st"] / a["n_st"]) if a["n_st"] > 0 else None,
    "sign_rate": a["sc"] / (a["n_st"] * dt) if a["n_st"] * dt >= 10.0 else None,
    "curve_ratio": a["ang_cur"] / a["des_cur"] if a["n_cur"] * dt >= MIN_CURVE_S and a["des_cur"] > 0 else None,
    "press_rate": a["press"] / act_min if act_min > 0 else None,
  }


def curve_step(curve_ratio):
  """Step that would bring the curve ratio to 1.0 if angle tracked P linearly, on the STEP grid, 1..3 steps."""
  need = abs(1.0 / curve_ratio - 1.0)
  return min(max(round(need / STEP) * STEP, STEP), MAX_STEP)


def update_state(state, stats, mode):
  """One drive's step. Returns the new state; `last` lists what each band did and why."""
  state = parse_state(state) if not isinstance(state, dict) else parse_state(json.dumps(state))
  f = list(state["factor"])
  prev = list(state["prev"])
  carry = [None] * len(BANDS)
  last = []
  for i, a in enumerate(stats.acc):
    m = band_metrics(a)
    k = BAND_NAMES[i]
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
    undo = -p["stepped"] if stepped_up else 0.0
    if stepped_up and p.get("sign_rate") and m["sign_rate"] > p["sign_rate"] * OSC_GROWTH:
      step, why = undo, f"revert: sign changes {p['sign_rate']:.2f} -> {m['sign_rate']:.2f}/s after the last increase"
    elif stepped_up and p.get("press_rate") is not None and press_now > p["press_rate"] * PRESS_GROWTH + PRESS_SLACK:
      step, why = undo, f"revert: override onsets {p['press_rate']:.2f} -> {press_now:.2f}/min after the last increase"
    elif m["sign_rate"] > SIGN_RATE_MAX:
      step, why = -STEP, f"down: sign changes {m['sign_rate']:.2f}/s > {SIGN_RATE_MAX}"
    elif m["curve_ratio"] is not None and m["curve_ratio"] > CURVE_RATIO_HIGH:
      step = -curve_step(m["curve_ratio"])
      why = f"down {-step:.2f}: curve ratio {m['curve_ratio']:.3f} > {CURVE_RATIO_HIGH}"
    elif (m["curve_ratio"] is not None and m["curve_ratio"] < CURVE_RATIO_LOW and m["sign_rate"] < SIGN_RATE_UP_MAX
          and press_now < PRESS_RATE_UP_MAX):
      step = curve_step(m["curve_ratio"])
      why = f"up {step:.2f}: curve ratio {m['curve_ratio']:.3f} < {CURVE_RATIO_LOW}"
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
# Trial building (offline). Values are the Lat*Scale* percent params (INT, Galaxy range 0-500, step 5).

SCHEMA_VERSION = 2
P_KEYS = tuple(f"LatPScale{n}" for n in BAND_NAMES)
I_KEYS = tuple(f"LatIScale{n}" for n in BAND_NAMES)
F_KEYS = tuple(f"LatFScale{n}" for n in BAND_NAMES)
BAND_KEYS = P_KEYS
SCHEDULE_KEY = "LatGainSchedule"


def _param_str(v):
  if isinstance(v, bytes):
    v = v.decode("utf-8", "replace")
  return "" if v is None else str(v)


def _pct(raw, default=100):
  try:
    return int(round(float(_param_str(raw).strip() or default)))
  except ValueError:
    return default


def band_gains(raw_params):
  """[{"p", "i", "f"}] per band: the Lat*Scale* percent params as stored (missing -> 100)."""
  return [{"p": _pct(raw_params.get(pk)), "i": _pct(raw_params.get(ik)), "f": _pct(raw_params.get(fk))}
          for pk, ik, fk in zip(P_KEYS, I_KEYS, F_KEYS, strict=True)]


def schedule_terms(raw):
  """Terms a LatGainSchedule value overrides ("p"/"i"/"f"), as LatControlPID parses it."""
  from openpilot.selfdrive.controls.lib.latcontrol_pid import parse_lat_gain_schedule
  try:
    sched = parse_lat_gain_schedule(_param_str(raw) or None)
  except Exception:
    sched = None
  return sorted(t for t in ("p", "i", "f") if sched and t in sched)


def strip_schedule_p(raw):
  """LatGainSchedule without its "p" term, so the LatPScale* bands drive P again. "" when nothing is left."""
  text = _param_str(raw).strip()
  if not text:
    return ""
  try:
    d = json.loads(text)
  except ValueError:
    return text
  if not isinstance(d, dict) or "p" not in d:
    return text
  d = {k: v for k, v in d.items() if k != "p"}
  return json.dumps(d, separators=(",", ":")) if any(t in d for t in ("i", "f")) else ""


def propose_p(current, factor):
  """current % x factor on the Galaxy 5 % grid; a step never rounds away to nothing."""
  if abs(factor - 1.0) < 1e-9:
    return current
  target = int(round(current * factor / PCT_STEP)) * PCT_STEP
  if factor > 1.0 and target <= current:
    target = current + PCT_STEP
  elif factor < 1.0 and target >= current:
    target = current - PCT_STEP
  return min(max(target, PCT_MIN), PCT_MAX)


def build_trial(stats, baseline, route_names, per_route, warnings):
  """One trial from one DriveStats over the selected routes. baseline = {"fingerprint", "gains", "raw"}."""
  state = update_state(default_state(baseline.get("fingerprint")), stats, APPLY_MODE)
  gains = baseline.get("gains") or band_gains({})
  bands = []
  for i, (name, lo, hi) in enumerate(BANDS):
    m = band_metrics(stats.acc[i]) if stats.acc[i]["n"] > 0 else {"min": 0.0, "straight_rms": None, "sign_rate": None,
                                                                   "curve_ratio": None, "press_rate": None}
    reason = state["last"][i] if i < len(state["last"]) else ""
    decision = reason.split(":", 2)[1].strip().split(" ")[0] if reason.count(":") >= 1 else "hold"
    cur = {k: int(gains[i][k]) for k in ("p", "i", "f")}
    bands.append({"name": name, "lowMph": lo, "highMph": hi, "pKey": P_KEYS[i],
                  "minutes": round(m["min"], 2), "ready": m["min"] >= MIN_MINUTES,
                  "signRate": m["sign_rate"], "curveRatio": m["curve_ratio"], "straightRms": m["straight_rms"],
                  "pressRate": m["press_rate"], "factor": state["factor"][i], "decision": decision, "reason": reason,
                  "current": cur, "proposed": {"p": propose_p(cur["p"], state["factor"][i]), "i": cur["i"], "f": cur["f"]}})
  raw = {k: _param_str(v) for k, v in baseline.get("raw", {}).items()}
  return {"schemaVersion": SCHEMA_VERSION, "bandNames": list(BAND_NAMES), "routeNames": list(route_names),
          "warnings": list(warnings), "baseline": {"fingerprint": baseline.get("fingerprint"), "gains": gains, "raw": raw,
                                                   "scheduleTerms": schedule_terms(raw.get(SCHEDULE_KEY))},
          "bands": bands, "factors": list(state["factor"]), "perRoute": list(per_route), "applied": None}


def build_band_params(trial):
  """{"LatPScaleLowSpeed": int, ...}: the P band params a trial writes. I/F are never written."""
  return {b["pKey"]: int(b["proposed"]["p"]) for b in trial["bands"]}


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
    self.tuning = {}      # TUNING_KEYS values from initData, str
    self.n = 0

  def frames(self):
    wanted = set(TUNING_KEYS)
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


def analyze_sources(sources, should_continue=None, on_progress=None, baseline_overrides=None):
  """sources: RouteLog list in analysis order (oldest first). Returns a trial dict (build_trial).

  baseline_overrides: {param: value} replacing the logged Lat*Scale* values the proposal starts from (what-if).
  The metrics still come from the logs, i.e. from the gains the routes were actually driven with."""
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
    per_route.setdefault(src.route, [0.0] * len(BANDS))
    for i in range(len(BANDS)):
      per_route[src.route][i] = round(per_route[src.route][i] + band_metrics(route_stats.acc[i])["min"], 2)
  route_names = list(dict.fromkeys(s.route for s in sources))
  latest = next((tuning_by_route[r] for r in reversed(route_names) if r in tuning_by_route), {})
  fps = {r: tuning_fingerprint(t) for r, t in tuning_by_route.items()}
  if len(set(fps.values())) > 1:
    warnings.append("routes were driven with different lateral tuning (fingerprint mismatch); the newest route's tuning is the baseline")
  if baseline_overrides:
    logged = dict(latest)
    latest = {**latest, **{k: _param_str(v) for k, v in baseline_overrides.items()}}
    for k, v in baseline_overrides.items():
      warnings.append(f"baseline override: {k} = {v} (routes were driven with {logged.get(k) or 'default'}; "
                      "the metrics describe that gain, not this one)")
  if "p" in schedule_terms(latest.get(SCHEDULE_KEY)):
    warnings.append("LatGainSchedule overrides P on these routes; the proposal is relative to the LatPScale* bands, "
                    "and applying removes the schedule's p term")
  baseline = {"fingerprint": tuning_fingerprint(latest) if latest else None, "gains": band_gains(latest), "raw": latest}
  return build_trial(stats, baseline, route_names, [{"route": r, "minutes": m} for r, m in per_route.items()], warnings)
