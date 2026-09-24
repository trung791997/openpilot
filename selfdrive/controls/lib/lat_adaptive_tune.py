"""On-device adaptive P trim for modified-EPS Honda lateral PID. PROTOTYPE, default off.

`LatAdaptiveTune`: 0 off (default), 1 shadow, 2 apply.

What it does
  While driving it only measures. Every engaged, hands-off frame above 4 m/s (no blinker, not
  steer-limited) is added to per-knot tracking statistics, split between the two neighbouring
  knots of KNOTS_MPH by linear weight, using the same definitions as tools/lateral/lat_pid_sim.py
  metrics(): straight = |desired| < 3 deg, curve = |desired| > 5 deg. The statistics are saved to
  `LatAdaptiveStats` once a minute.

  At the start of the next drive (controlsd start), `update_state` turns the previous drive's
  statistics (plus data carried from earlier drives for knots that had too little) into at most one STEP per knot of a multiplicative P factor, stored in
  `LatAdaptiveState`. In apply mode that factor multiplies the P trim for the whole drive; the gains
  never change during a drive. In shadow mode the factor is computed and stored but not applied,
  so it shows the direction the rules would take without acting on them.

Rules per knot (only with >= MIN_MINUTES of qualifying data; anything missing holds):
  1. The last step was up and straight sign changes rose > OSC_GROWTH or driver-override onsets
     rose > PRESS_GROWTH (+ PRESS_SLACK): step back down. Only in apply mode, where the step acted.
  2. Straight sign changes above SIGN_RATE_MAX: step down.
  3. Curve ratio (achieved/desired) above CURVE_RATIO_HIGH: step down.
  4. Curve ratio below CURVE_RATIO_LOW, sign changes under SIGN_RATE_UP_MAX and override onsets
     under PRESS_RATE_UP_MAX: step up.
  5. Otherwise hold.
  Factors are clamped to FACTOR_MIN..FACTOR_MAX and neighbouring knots to MAX_NEIGHBOUR_GAP.

Evidence: thresholds are checked against 19 logged routes (STATUS 116); replay evidence only, nothing
driven. It cannot tell a model-path error from a tracking error, and a driver-override onset from
EPS reaction torque (STATUS 114); both are why it moves slowly and only within +-15 %.
Clearing `LatAdaptiveState` resets it; `LatAdaptiveTune` 0 removes it from the loop entirely.

The factor is a trim on top of the manual tuning it was learned under, so the state stores a
fingerprint of TUNING_KEYS. When any of them differs at controlsd start, the factors reset to 1.0
and the previous drive's statistics are dropped. Without that, replay of routes 0000025f..0000026b
walked 40 mph to 1.15 while standard-band I was 25 (P covering an I shortfall) and then held it
there after I was raised to 75, because the curve ratio had moved back into the dead band (STATUS 116).
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

SAVE_EVERY_FRAMES = 6000
STATE_VERSION = 1
MODE_OFF, MODE_SHADOW, MODE_APPLY = 0, 1, 2

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
    stepped_up = p is not None and p.get("stepped", 0.0) > 0 and mode == MODE_APPLY
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


def _read_mode(params):
  try:
    mode = int(float(params.get("LatAdaptiveTune") or 0))
  except Exception:
    return MODE_OFF
  return mode if mode in (MODE_SHADOW, MODE_APPLY) else MODE_OFF


class LatAdaptiveTuner:
  """Glue for LatControlPID. All Params I/O is here; observe() is arithmetic only, plus one
  put_nonblocking a minute.

  The mode is live: LatControlPID calls refresh_mode() from its 300-frame (3 s) param refresh, so
  the Galaxy dropdown takes effect mid-drive like the Lat*Scale rows. Only the mode is live; the
  learned factors still step at most once, at the first activation in a drive. Switching shadow ->
  apply mid-drive applies the stored factor at once (a P step of at most +-15 %, same class as a
  live Lat*Scale edit); switching to off returns P to 1.0 at once and pauses learning."""
  def __init__(self, params):
    self.params = params
    self.mode = MODE_OFF
    self.state = default_state()
    self.stats = None
    self.frames = 0
    mode = _read_mode(params)
    if mode != MODE_OFF:
      self._start(mode, blocking=True)

  def _start(self, mode, blocking):
    """Load the state, take this drive's step from the previous drive's stats, start collecting.
    Runs once per drive, at the first activation (controlsd start, or mid-drive from Galaxy)."""
    put = self.params.put if blocking else self.params.put_nonblocking
    try:
      tuning = tuning_fingerprint({k: self.params.get(k) for k in TUNING_KEYS})
      self.state = parse_state(self.params.get("LatAdaptiveState") or "")
      prev_stats = DriveStats.from_json(self.params.get("LatAdaptiveStats") or "")
      dirty = False
      if self.state["tuning"] != tuning:
        # Manual tuning changed (or first start): what was learned was relative to other gains.
        reset = self.state["tuning"] is not None
        self.state = default_state(tuning)
        if reset:
          self.state["last"] = ["reset: manual lateral tuning changed"]
        dirty = True
      if prev_stats is not None and prev_stats.tuning != tuning:
        prev_stats = None  # collected under different gains, or the gains changed during that drive
      if prev_stats is not None and prev_stats.has_data():
        self.state = update_state(self.state, prev_stats, mode)
        dirty = True
      if dirty:
        put("LatAdaptiveState", json.dumps(self.state, separators=(",", ":")))
      put("LatAdaptiveStats", "")
      self.stats = DriveStats(self.state["carry"], tuning)
      self.mode = mode
    except Exception:
      self.mode = MODE_OFF
      self.state = default_state()
      self.stats = None

  def refresh_mode(self):
    """Re-read LatAdaptiveTune mid-drive. Off -> on starts learning (once per drive); on -> off
    saves what was collected and stops applying; shadow <-> apply only changes p_factor()."""
    mode = _read_mode(self.params)
    if mode == self.mode:
      return
    if mode == MODE_OFF:
      if self.stats is not None and self.stats.has_data():
        try:
          self.params.put_nonblocking("LatAdaptiveStats", self.stats.to_json())
        except Exception:
          pass
      self.mode = MODE_OFF
    elif self.stats is None:
      self._start(mode, blocking=False)
    else:
      self.mode = mode

  def p_factor(self, v_ego):
    if self.mode != MODE_APPLY:
      return 1.0
    return interp_factor(self.state["factor"], v_ego)

  def observe(self, v_ego, desired_deg, angle_deg, pressed, lane_change, steer_limited):
    if self.mode == MODE_OFF or self.stats is None:
      return
    self.stats.observe(v_ego, desired_deg, angle_deg, pressed, lane_change, steer_limited)
    self.frames += 1
    if self.frames % SAVE_EVERY_FRAMES == 0:
      try:
        self.params.put_nonblocking("LatAdaptiveStats", self.stats.to_json())
      except Exception:
        pass
