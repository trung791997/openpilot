"""A driver's hand on the wheel for the MetaDrive lateral sim (SIM_DRIVER), matching NRDR PID Lateral Tuning's offline
override sim (2026-09-28) so the two sims score the same presses.

The hand is a PI toward a wanted wheel angle with a neuromuscular lag, in STEER_TORQUE_SENSOR units (dt 0.01 s):
  e = want - meas (meas: wheel angle quantised to 0.1 deg), tqi = clip(tqi + ki e dt, +-3500),
  tq_target = clip(kd e - bd rate + tqi, +-3500), tq += dt / 0.15 (tq_target - tq); hands off: tq_target = tqi = 0.
kd 300 per deg, bd 8 per deg/s, ki 800 / 1500 / 3000 per deg s (soft / medium / firm).

want is set relative to the plan, controlsState.steeringAngleDesiredDeg. A route map cannot script the plan, so a press is
triggered by the live plan: once |plan| has stayed >= trigger_deg for dwell s, the press starts and lasts dur s with ramp r
(window(t, t0, t1, r) = clip((t - t0) / r, 0, 1) clip((t1 - t) / r, 0, 1)). Kinds:
  offset: want = plan - sign(plan) offset_deg window   (curve hug / nudge; outward = against the curve)
  scale:  want = plan (1 - scale window)                (turns: 0.3 holds 70% of the plan, cf. min(plan, 90 - 27 w))

  widen:  want = plan + sign(plan) offset_deg window     (driver asks for more than the plan)
  push:   no hand loop; tq_extra = sign amp_tq window      (constant push centring check, amp signed)
  rest:   no hand loop; tq_extra = 250 sin(4.4 t) + 100 sin(13.2 t + 1) over the window (resting hand)
A press starts on the plan trigger, once v_ego >= "trigger_v" for dwell s, or at "at_s" seconds after the driver's first engaged step when given.

"version": "v2" (Kevin's low-speed driver, 2026-09-28): the loop through the assist is capped near 7 rad/s with zeta ~0.8:
s = min(1, 49000 / g(v) / kd), kd and ki scaled by s, bd = max(bd, 11200 / g(v)) where s < 1. At 45 mph s = 1 (v1).

SIM_DRIVER is JSON: {"version": "v2", "ki": 800, "presses": [{"kind": "offset", "offset_deg": 3, "trigger_deg": 8,
"dwell": 1.0, "dur": 4.0, "r": 0.4}]}. Each press fires once, in order. Sim evidence only.
"""
import json
import math

import numpy as np

DT = 0.01
KD = 300.0
BD = 8.0
TQ_MAX = 3500.0
LAG_S = 0.15
ANGLE_QUANT_DEG = 0.1
KI = {"soft": 800.0, "medium": 1500.0, "firm": 3000.0}

# How driver torque moves the wheel (added to the plant's rate equation, undelayed, no assist table): deg/s^2 per 1000
# sensor units. Fitted by free-run on routes 290 and 292 (lat active, |tq| >= 600, |angle| <= 90): 100-150 above 11 m/s
# (rms 1.55 -> 1.2 deg); 800 below 11 m/s is the best but weak fit (rms 5 deg).
G_BP = (8.0, 14.0)
G_V = (800.0, 120.0)


def driver_gain(v_ego: float) -> float:
  return float(np.interp(v_ego, G_BP, G_V))


def window(t: float, t0: float, t1: float, r: float) -> float:
  return min(max((t - t0) / r, 0.0), 1.0) * min(max((t1 - t) / r, 0.0), 1.0)


class DriverModel:
  def __init__(self, spec: str | dict):
    s = json.loads(spec) if isinstance(spec, str) else spec
    ki = s.get("ki", "soft")
    self.ki = KI[ki] if isinstance(ki, str) else float(ki)
    self.version = s.get("version", "v1")
    self.presses = list(s.get("presses", []))
    self.t = 0.0
    self.tq = 0.0
    self.tqi = 0.0
    self.above_s = 0.0
    self.active = None  # (press, t0, t1)
    self.log = []  # (t, t0, t1) of each press, for scoring

  def want(self, plan: float) -> float | None:
    if self.active is None:
      return None
    p, t0, t1 = self.active
    w = window(self.t, t0, t1, p.get("r", 0.4))
    if p["kind"] in ("push", "rest"):
      return None
    if p["kind"] == "offset":
      return plan - math.copysign(p["offset_deg"], plan) * w
    if p["kind"] == "widen":
      return plan + math.copysign(p["offset_deg"], plan) * w
    return plan * (1.0 - p["scale"] * w)

  def tq_extra(self) -> float:
    if self.active is None:
      return 0.0
    p, t0, t1 = self.active
    w = window(self.t, t0, t1, p.get("r", 0.3))
    if p["kind"] == "push":
      return p["amp_tq"] * w
    if p["kind"] == "rest":
      return (250.0 * math.sin(4.4 * self.t) + 100.0 * math.sin(13.2 * self.t + 1.0)) * w
    return 0.0

  def gains(self, v_ego: float) -> tuple[float, float, float]:
    kd, ki, bd = KD, self.ki, BD
    if self.version == "v2":
      g = driver_gain(v_ego)
      sc = min(1.0, 49.0 * 1000.0 / g / kd)
      if sc < 1.0:
        kd, ki, bd = kd * sc, ki * sc, max(bd, 11.2 * 1000.0 / g)
    return kd, ki, bd

  def update(self, plan: float, angle: float, rate: float, v_ego: float = 20.0) -> float:
    """One 100 Hz step: plan (deg), true wheel angle (deg) and rate (deg/s). Returns the driver's torque."""
    self.t += DT
    if self.active is not None and self.t >= self.active[2]:
      self.active = None
    if self.active is None and self.presses:
      p = self.presses[0]
      if "at_s" in p:
        go = self.t >= p["at_s"]
      elif "trigger_v" in p:  # straights (constant push): once at speed for dwell s
        self.above_s = self.above_s + DT if v_ego >= p["trigger_v"] else 0.0
        go = self.above_s >= p.get("dwell", 1.0)
      else:
        self.above_s = self.above_s + DT if abs(plan) >= p["trigger_deg"] else 0.0
        go = self.above_s >= p.get("dwell", 1.0)
      if go:
        self.presses.pop(0)
        self.active = (p, self.t, self.t + p["dur"])
        self.log.append((self.t, self.t + p["dur"]))
    want = self.want(plan)
    if want is None:
      tq_target = self.tqi = 0.0
    else:
      kd, ki, bd = self.gains(v_ego)
      e = want - round(angle / ANGLE_QUANT_DEG) * ANGLE_QUANT_DEG
      self.tqi = float(np.clip(self.tqi + ki * e * DT, -TQ_MAX, TQ_MAX))
      tq_target = float(np.clip(kd * e - bd * rate + self.tqi, -TQ_MAX, TQ_MAX))
    self.tq += DT / LAG_S * (tq_target - self.tq)
    return self.tq + self.tq_extra()
