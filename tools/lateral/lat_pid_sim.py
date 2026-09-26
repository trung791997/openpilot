#!/usr/bin/env python3
"""Offline lateral PID tuning simulator for modified-EPS Honda PID lateral.

Three stages, each checked against the log before the next one is trusted:

  1. extract   rlogs -> per-frame arrays at controlsState rate (cached as .npz next to the route,
               never in the repo).
  2. replay    open loop: the real LatControlPID is fed the logged inputs (carState, liveParameters,
               controlsState.desiredCurvature, active flag) with the logged tuning params, and its
               output is compared with the logged pidState.output. This proves the harness reproduces
               the on-car controller before anything is changed.
  3. sim       closed loop: the same controller drives an identified steering plant (torque command ->
               steering angle), with the desired curvature held to the logged trajectory. Tuning
               overrides (LatIScaleStandard=75, HondaLateralPidKpScale=0.65, ...) are applied to the
               controller only.

The plant is fitted from logs (see fit_plant). It is a speed-scheduled second-order rack model with
a command delay (fitted or --delay) and 0.1 deg angle quantisation. Its validity is judged by `validate`:
a free-run of the plant on the logged command,
and a closed-loop run at the logged settings whose tracking metrics must match the logged ones.

Limits (read before trusting a number):
  - Desired curvature is exogenous. On the car, a worse tracker changes the car's heading, and the
    model and lane centering then ask for a different curvature. The sim does not close that loop, so
    it scores how well the rack follows a fixed request, not lane position.
  - The plant is linear-in-parameters and fitted on ordinary driving. It will not predict behaviour
    outside what the fit saw (limit cycles at unusual gains, torque saturation, low grip).
  - Evidence from this tool is replay/sim evidence only.

Usage:
  python tools/lateral/lat_pid_sim.py replay ROUTE_DIR
  python tools/lateral/lat_pid_sim.py fit ROUTE_DIR [ROUTE_DIR ...] --out plant.json
  python tools/lateral/lat_pid_sim.py validate --plant plant.json ROUTE_DIR [...]
  python tools/lateral/lat_pid_sim.py sim --plant plant.json ROUTE_DIR [...] --set LatIScaleStandard=75 [--set K=V ...]
  python tools/lateral/lat_pid_sim.py sweep --plant plant.json ROUTE_DIR [...] --param LatIScaleStandard --values 25,50,75,100
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from types import SimpleNamespace

import numpy as np

DT = 0.01
MPH = 0.44704
BANDS = (("low <25mph", 0.0, 25 * MPH), ("standard 25-50", 25 * MPH, 50 * MPH), ("highway >50", 50 * MPH, 99.0))

# Params the controller reads through Params() every 300 frames, plus the Kp/Ki toggles.
TUNING_KEYS = (
  "LatPScaleLowSpeed", "LatPScaleStandard", "LatPScaleHighway",
  "LatIScaleLowSpeed", "LatIScaleStandard", "LatIScaleHighway",
  "LatFScaleLowSpeed", "LatFScaleStandard", "LatFScaleHighway", "LatGainSchedule",
  "HondaCenterScale", "HondaCenterBoostThreshold", "HondaCenterBoostMinSpeed",
  "NrdrLatAngleRateLimit", "HondaTorqueLowPassFilter",
  "HondaLpfTauLowSpeed", "HondaLpfTauStandard", "HondaLpfTauHighway", "NrdrLatUseFirmwareVgr",
  "HondaLateralPidKpScale", "HondaLateralPidKiScale",
  "NrdrLearnSteerRatio", "NrdrLearnStiffness", "NrdrLearnAngleOffset",
  # carcontroller steering path (opendbc_repo/opendbc/car/honda/carcontroller.py _update_steering_torque)
  "NrdrMinSteerSpeed", "HondaOverrideTorqueScale", "HondaOverrideFadeUpSecs", "HondaOverrideFadeDownSecs",
  "HondaSteerDeltaLimiter", "HondaSteerDeltaUp", "HondaSteerDeltaDown",
)

FIELDS = ("t", "v", "a_ego", "angle", "rate", "pressed", "eps_torque", "lblink", "rblink",
          "sr", "stiff", "offset", "roll", "des_curv", "active", "out", "des_angle",
          "cc_torque", "co_torque", "lane_change", "log_p", "log_i", "log_f")


# ----------------------------------------------------------------------------------------------
# 1. extract

def _segments(route_dir):
  segs = glob.glob(os.path.join(route_dir, "*", "rlog.zst")) + glob.glob(os.path.join(route_dir, "*", "rlog.bz2"))
  return sorted(segs, key=lambda p: int(os.path.basename(os.path.dirname(p))))


def extract(route_dir, cache=True):
  cache_path = os.path.join(route_dir, "lat_pid_sim.npz")
  if cache and os.path.exists(cache_path):
    z = np.load(cache_path, allow_pickle=False)
    data = {k: z[k] for k in FIELDS}
    data["cp_bytes"] = z["cp_bytes"].tobytes()
    data["params"] = json.loads(str(z["params"]))
    data["route"] = os.path.basename(os.path.normpath(route_dir))
    return data

  from openpilot.tools.lib.logreader import LogReader
  rows = []
  cp_bytes = None
  params = {}
  cs = lp = cc = co = None
  lcs = 0
  t0 = None
  for seg in _segments(route_dir):
    try:
      lr = LogReader(seg)
      for m in lr:
        w = m.which()
        if t0 is None:
          t0 = m.logMonoTime
        if w == "initData" and not params:
          for e in m.initData.params.entries:
            if e.key in TUNING_KEYS:
              try:
                params[e.key] = e.value.decode()
              except Exception:
                pass
        elif w == "carParams" and cp_bytes is None:
          cp_bytes = m.carParams.as_builder().to_bytes()
        elif w == "carState":
          cs = m.carState
        elif w == "liveParameters":
          lp = m.liveParameters
        elif w == "carControl":
          cc = m.carControl
        elif w == "carOutput":
          co = m.carOutput
        elif w == "modelV2":
          lcs = int(m.modelV2.meta.laneChangeState.raw)
        elif w == "controlsState" and cs is not None and lp is not None:
          c = m.controlsState
          lcs_ = c.lateralControlState
          if lcs_.which() != "pidState":
            continue
          p = lcs_.pidState
          rows.append((
            (m.logMonoTime - t0) * 1e-9, cs.vEgo, cs.aEgo, cs.steeringAngleDeg, cs.steeringRateDeg,
            float(cs.steeringPressed), cs.steeringTorque, float(cs.leftBlinker), float(cs.rightBlinker),
            lp.steerRatio, lp.stiffnessFactor, lp.angleOffsetDeg, lp.roll, c.desiredCurvature,
            float(p.active), p.output, p.steeringAngleDesiredDeg,
            cc.actuators.torque if cc is not None else 0.0,
            co.actuatorsOutput.torque if co is not None else 0.0, float(lcs), p.p, p.i, p.f,
          ))
    except Exception as e:  # a truncated final segment is common; keep what was read
      print(f"warning: {seg}: {e}", file=sys.stderr)
  if cp_bytes is None:
    raise RuntimeError(f"{route_dir}: no carParams in log")
  arr = np.array(rows, dtype=np.float64)
  data = {k: arr[:, i] for i, k in enumerate(FIELDS)}
  data["cp_bytes"] = cp_bytes
  data["params"] = params
  data["route"] = os.path.basename(os.path.normpath(route_dir))
  if cache:
    np.savez_compressed(cache_path, **{k: data[k] for k in FIELDS},
                        cp_bytes=np.frombuffer(cp_bytes, dtype=np.uint8), params=json.dumps(params))
  return data


# ----------------------------------------------------------------------------------------------
# controller harness

class _DictParams:
  """Stands in for Params() inside LatControlPID; values are strings as on the device."""
  def __init__(self, values):
    self.values = values

  def get(self, key, *args, **kwargs):
    v = self.values.get(key)
    return None if v is None else str(v)

  def get_bool(self, key, *args, **kwargs):
    v = self.values.get(key)
    if v is None:
      return bool(kwargs.get("default", False))
    return str(v).strip().lower() in ("1", "true")


def _truthy(v, default=True):
  if v is None:
    return default
  return str(v).strip().lower() in ("1", "true")


class Controller:
  """The real LatControlPID plus the VehicleModel update controlsd does before calling it."""
  def __init__(self, cp_bytes, params, testing_ground=False):
    from cereal import car, custom
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.vehicle_model import VehicleModel
    from openpilot.selfdrive.controls.lib import latcontrol_pid
    from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID

    with car.CarParams.from_bytes(cp_bytes) as r:
      self.CP = r.as_builder().as_reader()
    fpcp = custom.StarPilotCarParams.new_message().as_reader()
    self.CI = interfaces[self.CP.carFingerprint](self.CP, fpcp)
    self.VM = VehicleModel(self.CP)
    self.params = dict(params)
    # The Civic Bosch "testing ground" slot is chosen by a file on the device that the log does not
    # record. The replay stage shows whether forcing it off reproduces the logged output.
    self._lp = latcontrol_pid
    self._saved = (latcontrol_pid.civic_bosch_modified_lateral_testing_ground_active, latcontrol_pid.Params)
    latcontrol_pid.civic_bosch_modified_lateral_testing_ground_active = lambda *a, **kw: testing_ground
    latcontrol_pid.Params = lambda: _DictParams(self.params)
    try:
      self.lac = LatControlPID(self.CP, self.CI, DT)
    finally:
      latcontrol_pid.Params = self._saved[1]
    self.lac.params = _DictParams(self.params)
    self.toggles = SimpleNamespace(
      honda_lateral_pid_kp_scale=float(self.params.get("HondaLateralPidKpScale", 1.0)),
      honda_lateral_pid_ki_scale=float(self.params.get("HondaLateralPidKiScale", 1.0)),
    )
    self.learn_sr = _truthy(self.params.get("NrdrLearnSteerRatio"))
    self.learn_x = _truthy(self.params.get("NrdrLearnStiffness"))
    self.learn_offset = _truthy(self.params.get("NrdrLearnAngleOffset"))

  def close(self):
    self._lp.civic_bosch_modified_lateral_testing_ground_active = self._saved[0]

  def step(self, d, k, angle, rate, steer_limited):
    x = max(d["stiff"][k] if self.learn_x else 1.0, 0.1)
    sr = max(d["sr"][k] if self.learn_sr else self.CP.steerRatio, 0.1)
    self.VM.update_params(x, sr)
    lp = SimpleNamespace(angleOffsetDeg=float(d["offset"][k]) if self.learn_offset else 0.0, roll=float(d["roll"][k]))
    CS = SimpleNamespace(
      vEgo=float(d["v"][k]), aEgo=float(d["a_ego"][k]), steeringAngleDeg=float(angle), steeringRateDeg=float(rate),
      steeringPressed=bool(d["pressed"][k]), steeringTorque=float(d["eps_torque"][k]),
      leftBlinker=bool(d["lblink"][k]), rightBlinker=bool(d["rblink"][k]), standstill=d["v"][k] < 0.1,
    )
    out, des, self.last_log = self.lac.update(bool(d["active"][k]), CS, self.VM, lp, bool(steer_limited), float(d["des_curv"][k]),
                                  False, 0.0, None, None, self.toggles)
    return float(out), float(des)


# ----------------------------------------------------------------------------------------------
# 2. replay (open loop)

def replay(d, overrides=None, testing_ground=False):
  params = dict(d["params"])
  params.update(overrides or {})
  ctl = Controller(d["cp_bytes"], params, testing_ground)
  n = len(d["t"])
  out = np.zeros(n)
  des = np.zeros(n)
  try:
    for k in range(n):
      steer_limited = abs(d["cc_torque"][k] - d["co_torque"][k]) > 1e-2
      out[k], des[k] = ctl.step(d, k, d["angle"][k], d["rate"][k], steer_limited)
  finally:
    ctl.close()
  return out, des


# ----------------------------------------------------------------------------------------------
# replay with the logged integrator freeze
#
# The integrator freezes on steer_limited_by_safety, which controlsd derives from carControl vs the
# latest carOutput. The log interleaves those messages too coarsely to recover which frame each
# comparison used, so the check below takes the freeze pattern from the log itself (logged I
# unchanged from the previous frame). Everything else is recomputed.

def replay_logged_freeze(d, overrides=None, testing_ground=False):
  params = dict(d["params"])
  params.update(overrides or {})
  ctl = Controller(d["cp_bytes"], params, testing_ground)
  li = d["log_i"]
  frozen = np.r_[False, li[1:] == li[:-1]]
  out = np.zeros(len(li))
  try:
    for k in range(len(li)):
      out[k], _ = ctl.step(d, k, d["angle"][k], d["rate"][k], frozen[k])
  finally:
    ctl.close()
  return out


def replay_report(d, overrides=None):
  act = d["active"] > 0.5
  print(f"== {d['route']}  replay (open loop)  {act.sum() / 100 / 60:.1f} min active  overrides {overrides or {}}")
  for label, out in (("logged tuning params, freeze recomputed", replay(d, overrides)[0]),
                     ("logged tuning params, logged freeze", replay_logged_freeze(d, overrides))):
    e = out[act] - d["out"][act]
    print(f"  {label:42s} torque |sim-log| median {np.median(abs(e)):.1e}  p99 {np.percentile(abs(e), 99):.1e}" +
          f"  (logged |out| median {np.median(abs(d['out'][act])):.3f})")


# ----------------------------------------------------------------------------------------------
# 3. plant: delivered torque command -> steering angle
#
#   rate' = (c0 + c1 v) rate + (c2 + c3 v^2/100) theta + (c4 + c5 v + c6 v^2/100) u + c7 + c8 tanh(rate / 2)
#   theta' = rate
#
# theta is the steering angle minus paramsd's learned offset, rate the logged steering rate (it
# tracks a 50 ms angle derivative at r = 0.96; the angle itself is 0.1 deg quantised), u the
# delivered command (carOutput.actuatorsOutput.torque). Delay 0: a delay grid 0-250 ms fitted worst
# at every nonzero delay on held-out data. Fitted by Levenberg-Marquardt on free-run angle error over
# 1 s windows of engaged, hands-off driving above 4 m/s. The driver torque sensor is deliberately
# not an input: it carries the logged trajectory and would bias any comparison towards the tuning
# that was actually driven.

PLANT_WIN = 100
PLANT_STRIDE = 50


# The command reaches the rack DELAY frames late. Without it the fitted plant answered the command in the
# same frame, the closed loop had more phase margin than the car, and the sim under-predicted weave
# (sign changes 0.5-0.6 /s against 0.8 logged, item 113). fit_plant picks the delay from FIT_DELAYS by
# free-run residual. The angle sensor reports 0.1 deg steps (STEER_ANGLE factor), so the controller and
# the metrics see the quantised angle, as on the car. STATUS 132: most of that gap was the quantisation
# (raw zero crossings count sensor steps); with the analyzer's deadband the undelayed plant already matched
# the logs, and 5 frames of delay matches both counts best.
FIT_DELAYS = (0, 2, 4, 6, 8, 10, 12, 15, 20)
ANGLE_QUANT_DEG = 0.1
SIGN_HYST_DEG = 0.15   # lat_tune_analyzer.SIGN_HYST_DEG


class Plant:
  def __init__(self, coef, delay=0, quant=0.0):
    self.c = np.asarray(coef, dtype=float)
    self.delay = int(delay)
    self.quant = float(quant)

  @staticmethod
  def from_json(j):
    # Plants fitted before the delay existed carry neither key and keep their old, undelayed behaviour.
    return Plant(j["coef"], j.get("delay_frames", 0), j.get("angle_quant_deg", 0.0))

  def measure(self, theta):
    return float(np.round(theta / self.quant) * self.quant) if self.quant > 0 else theta

  def accel(self, theta, rate, u, v):
    c = self.c
    return ((c[0] + c[1] * v) * rate + (c[2] + c[3] * v * v / 100) * theta
            + (c[4] + c[5] * v + c[6] * v * v / 100) * u + c[7] + c[8] * np.tanh(rate / 2.0))

  def to_json(self):
    return {"coef": self.c.tolist(), "delay_frames": self.delay, "angle_quant_deg": self.quant,
            "model": "rate' = (c0+c1 v) r + (c2+c3 v^2/100) th + (c4+c5 v+c6 v^2/100) u(t - delay) + c7 + c8 tanh(r/2)"}


def _hands_off(d):
  return (d["active"] > 0.5) & (d["pressed"] < 0.5) & (d["v"] > 4.0)


def _windows(d, win, stride):
  m = _hands_off(d)
  cs = np.r_[0, np.cumsum(~m)]
  st = np.arange(1, len(m) - win - 1, stride)
  return st[(cs[st + win + 1] - cs[st]) == 0]


def _pack(ds, win, stride):
  """Windows of `win` frames; "u" also holds the max(FIT_DELAYS) delivered commands before each window."""
  pre = max(FIT_DELAYS)
  parts = []
  for d in ds:
    st = _windows(d, win, stride)
    st = st[st >= pre]
    if len(st) == 0:
      continue
    idx = st[:, None] + np.arange(win + 1)[None, :]
    uidx = st[:, None] + np.arange(-pre, win + 1)[None, :]
    parts.append({"v": d["v"][idx], "u": d["co_torque"][uidx], "th": d["angle"][idx] - d["offset"][idx], "r": d["rate"][idx]})
  return {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}


def _plant_freerun(p, plant):
  th = p["th"][:, 0].copy()
  r = p["r"][:, 0].copy()
  out = np.zeros_like(p["th"])
  out[:, 0] = th
  u0 = max(FIT_DELAYS) - plant.delay
  for j in range(p["th"].shape[1] - 1):
    r = r + plant.accel(th, r, p["u"][:, u0 + j], p["v"][:, j]) * DT
    th = th + r * DT
    out[:, j + 1] = th
  return out


def fit_plant(ds, iters=60, verbose=True, delays=FIT_DELAYS):
  """Fits the coefficients at each candidate delay and keeps the delay with the lowest free-run residual."""
  p = _pack(ds, PLANT_WIN, PLANT_STRIDE)
  best = None
  for delay in delays:
    plant, cost = _fit_coef(p, delay, iters, verbose)
    if verbose:
      print(f"  delay {delay:2d} frames: window rms {np.sqrt(cost):.3f} deg")
    if best is None or cost < best[1]:
      best = (plant, cost)
  best[0].quant = ANGLE_QUANT_DEG
  return best[0], p["v"].shape[0]


def _fit_coef(p, delay, iters, verbose):
  def resid(c):
    return (_plant_freerun(p, Plant(c, delay)) - p["th"])[:, 10::10].ravel()

  c = np.array([-8.0, 0.0, -2.0, 0.0, 300.0, 0.0, 0.0, 0.0, 0.0])
  lam = 1e-2
  r0 = resid(c)
  for it in range(iters):
    J = np.zeros((len(r0), len(c)))
    for i in range(len(c)):
      h = 1e-4 * max(1.0, abs(c[i]))
      cc = c.copy()
      cc[i] += h
      J[:, i] = (resid(cc) - r0) / h
    A = J.T @ J
    g = J.T @ r0
    improved = False
    while lam < 1e8:
      cn = c + np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -g)
      rn = resid(cn)
      if np.isfinite(rn).all() and rn @ rn < r0 @ r0:
        rel = 1 - (rn @ rn) / (r0 @ r0)
        c, r0, lam, improved = cn, rn, max(lam / 3, 1e-6), True
        break
      lam *= 5
    if verbose > 1:
      print(f"  fit it {it:2d}  window rms {np.sqrt(np.mean(r0 ** 2)):.3f} deg")
    if not improved or rel < 1e-6:
      break
  return Plant(c, delay), float(np.mean(r0 ** 2))


def plant_holdout(ds, plant, win=300, stride=100):
  """Free-run error at the end of `win`-frame windows, and the error of just holding the start angle."""
  p = _pack(ds, win, stride)
  s = _plant_freerun(p, plant)
  return (float(np.sqrt(np.mean((s[:, -1] - p["th"][:, -1]) ** 2))),
          float(np.sqrt(np.mean((p["th"][:, 0] - p["th"][:, -1]) ** 2))), p["v"].shape[0])


# ----------------------------------------------------------------------------------------------
# 4. closed loop

class CarControllerSteer:
  """The steering-torque stages of the Honda carcontroller that can make the delivered command differ
  from CC.actuators.torque: minimum steer speed, the driver-override ramp and the optional delta limiter.
  Mirrors CarController._update_steering_torque with NrdrIncreaseOverrideTolerance off (raw steeringPressed)."""
  def __init__(self, params):
    def g(k, dflt):
      return float(params.get(k, dflt))
    self.min_speed = min(max(g("NrdrMinSteerSpeed", 1), 0), 45) * MPH
    self.override_scale = min(max(g("HondaOverrideTorqueScale", 0), 0), 100) / 100.0
    self.fade_up = g("HondaOverrideFadeUpSecs", 1.0)
    self.fade_down = g("HondaOverrideFadeDownSecs", 0.0)
    self.delta = _truthy(params.get("HondaSteerDeltaLimiter"), default=False)
    self.up = g("HondaSteerDeltaUp", 3.0)
    self.down = g("HondaSteerDeltaDown", 3.0)
    self.ramp = 0.0
    self.last = 0.0
    self.active_prev = False

  def step(self, cmd, active, v, pressed):
    cmd = cmd if active else 0.0
    if v < self.min_speed:
      cmd = 0.0
    if active:
      if not self.active_prev:
        self.ramp = 0.0
      if pressed:
        self.ramp = self.override_scale if self.fade_down <= 0 else max(self.override_scale, self.ramp - DT / self.fade_down)
      else:
        self.ramp = 1.0 if self.fade_up <= 0 else min(1.0, self.ramp + DT / self.fade_up)
      cmd *= self.ramp
    else:
      self.ramp = 0.0
    if self.delta:
      cmd = min(max(cmd, self.last - self.down * DT), self.last + self.up * DT)
    self.last = cmd
    self.active_prev = active
    return cmd


def simulate(d, plant, overrides=None, testing_ground=False):
  """Closed loop. While engaged, hands off and above 4 m/s the plant integrates the delivered command;
  everywhere else the state is re-synced to the log (the driver, or nobody, is steering)."""
  params = dict(d["params"])
  params.update(overrides or {})
  ctl = Controller(d["cp_bytes"], params, testing_ground)
  ccs = CarControllerSteer(params)
  n = len(d["t"])
  ang = np.zeros(n)
  des = np.zeros(n)
  out = np.zeros(n)
  deliv = np.zeros(n)
  sim_on = _hands_off(d)
  th = d["angle"][0]
  r = d["rate"][0]
  steer_limited = False
  try:
    for k in range(n):
      if not sim_on[k]:
        th, r = d["angle"][k], d["rate"][k]
      ang[k] = plant.measure(th) if sim_on[k] else th
      out[k], des[k] = ctl.step(d, k, ang[k], r, steer_limited)
      deliv[k] = ccs.step(out[k], d["active"][k] > 0.5, d["v"][k], d["pressed"][k] > 0.5)
      steer_limited = (d["active"][k] > 0.5) and abs(out[k] - deliv[k]) > 1e-2
      if sim_on[k]:
        th_rel = th - d["offset"][k]
        r = r + plant.accel(th_rel, r, deliv[max(k - plant.delay, 0)], d["v"][k]) * DT
        th = th + r * DT
  finally:
    ctl.close()
  return ang, des, out, deliv


def metrics(d, ang, des, mask=None):
  """Same definitions as the owner-facing lateral report: per speed band, engaged and hands off."""
  m0 = _hands_off(d) if mask is None else mask
  v = d["v"]
  res = {}
  for name, lo, hi in BANDS:
    m = m0 & (v >= max(lo, 4.0)) & (v < hi)
    if m.sum() < 3000:
      res[name] = None
      continue
    e = des[m] - ang[m]
    cur = m & (np.abs(des) > 5)
    st = m & (np.abs(des) < 3)
    es = des[st] - ang[st]
    res[name] = {
      "min": m.sum() / 100 / 60,
      "err_rms": float(np.sqrt(np.mean(e ** 2))),
      "bias": float(e.mean()),
      "curve_ratio": float(np.median(ang[cur] / des[cur])) if cur.sum() > 300 else None,
      "curve_s": cur.sum() / 100,
      "straight_rms": float(np.sqrt(np.mean(es ** 2))) if st.sum() else None,
      "zero_cross": float(np.sum(np.diff(np.sign(es)) != 0) / (st.sum() / 100)) if st.sum() else None,
      "sign_hyst": sign_changes(des - ang, st) / (st.sum() / 100) if st.sum() else None,
      "lag_s": tracking_lag(des, ang, cur),
    }
  return res


def sign_changes(err, straight):
  """Straight-frame error sign changes with the analyzer's deadband: a change needs a swing from >= +h to <= -h
  (or back), and the count restarts at every frame that is not straight, as DriveStats.observe does."""
  prev = 0
  n = 0
  for e, s in zip(err, straight, strict=True):
    if not s:
      prev = 0
      continue
    sign = 1 if e >= SIGN_HYST_DEG else -1 if e <= -SIGN_HYST_DEG else 0
    if sign and prev and sign != prev:
      n += 1
    if sign:
      prev = sign
  return n


def tracking_lag(des, ang, mask, max_frames=60):
  """Shift (s) of the angle behind the desired that minimises the curve-frame error, or None."""
  idx = np.where(mask)[0]
  idx = idx[idx + max_frames < len(ang)]
  if len(idx) < 300:
    return None
  costs = [np.mean((des[idx] - ang[idx + k]) ** 2) for k in range(max_frames + 1)]
  return int(np.argmin(costs)) * DT


def _fmt(x, f):
  return "  -  " if x is None else format(x, f)


def print_metrics(label, res):
  print(f"  {label}")
  for name, r in res.items():
    if r is None:
      print(f"    {name:15s} too little data")
      continue
    print(f"    {name:15s} {r['min']:4.1f} min | err rms {r['err_rms']:5.2f} deg  bias {r['bias']:+.2f} | curve actual/des " +
          f"{_fmt(r['curve_ratio'], '.3f')} ({r['curve_s']:.0f} s) | straight rms {_fmt(r['straight_rms'], '.2f')}" +
          f"  sign changes {_fmt(r['zero_cross'], '.1f')}/s ({_fmt(r['sign_hyst'], '.2f')}/s at {SIGN_HYST_DEG} deg)" +
          f" | lag {_fmt(r['lag_s'], '.2f')} s")


# ----------------------------------------------------------------------------------------------

def load(route_dirs):
  return [extract(r) for r in route_dirs]


def _overrides(pairs):
  out = {}
  for p in pairs or []:
    k, _, v = p.partition("=")
    out[k.strip()] = v.strip()
  return out


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest="cmd", required=True)
  p = sub.add_parser("replay", help="open-loop check of the controller harness against the log")
  p.add_argument("routes", nargs="+")
  p.add_argument("--set", action="append", help="param override KEY=VALUE, e.g. HondaLateralPidKpScale=1.0")
  p = sub.add_parser("fit", help="fit the steering plant")
  p.add_argument("routes", nargs="+")
  p.add_argument("--out", required=True)
  p.add_argument("--delay", type=int, help="fit at this command delay (frames) instead of picking from FIT_DELAYS; "
                 "on 263/268/271 the free-run picks 0 but 5 matches the closed-loop sign changes best (STATUS 132)")
  p = sub.add_parser("validate", help="plant free-run and closed-loop-at-logged-settings vs the log")
  p.add_argument("routes", nargs="+")
  p.add_argument("--plant", required=True)
  p.add_argument("--set", action="append", help="param override applied to every route (e.g. an ineffective Kp scale)")
  p = sub.add_parser("sim", help="closed loop with tuning overrides")
  p.add_argument("routes", nargs="+")
  p.add_argument("--plant", required=True)
  p.add_argument("--base", action="append", help="override applied to both runs (e.g. the effective Kp scale)")
  p.add_argument("--set", action="append", required=True)
  p = sub.add_parser("sweep", help="closed loop over values of one param")
  p.add_argument("routes", nargs="+")
  p.add_argument("--plant", required=True)
  p.add_argument("--base", action="append")
  p.add_argument("--param", required=True)
  p.add_argument("--values", required=True)
  args = ap.parse_args(argv)

  if args.cmd == "replay":
    for d in load(args.routes):
      replay_report(d, _overrides(args.set))
    return

  if args.cmd == "fit":
    ds = load(args.routes)
    plant, nwin = fit_plant(ds, delays=FIT_DELAYS if args.delay is None else (args.delay,))
    j = plant.to_json()
    j.update({"routes": [d["route"] for d in ds], "windows": int(nwin), "window_s": PLANT_WIN * DT})
    with open(args.out, "w") as f:
      json.dump(j, f, indent=1)
    print(f"plant written to {args.out}: delay {plant.delay} frames, {np.round(plant.c, 3).tolist()}")
    return

  with open(args.plant) as f:
    pj = json.load(f)
  plant = Plant.from_json(pj)
  ds = load(args.routes)

  if args.cmd == "validate":
    ov = _overrides(args.set)
    print(f"plant fitted on {pj.get('routes')}, delay {plant.delay} frames, angle step {plant.quant} deg")
    for d in ds:
      e, hold, n = plant_holdout([d], plant)
      held = "" if d["route"] in pj.get("routes", []) else "  (held out)"
      print(f"== {d['route']}{held}  plant free-run 3 s end error {e:.2f} deg (hold-angle baseline {hold:.2f}, {n} windows)")
      print_metrics("logged", metrics(d, d["angle"], d["des_angle"]))
      ang, des, _, _ = simulate(d, plant, ov)
      print_metrics(f"sim at logged settings {ov or ''}", metrics(d, ang, des))
    return

  base = _overrides(args.base)
  if args.cmd == "sim":
    variants = [("baseline " + str(base or "logged"), base), ("with " + str(_overrides(args.set)), {**base, **_overrides(args.set)})]
  else:
    variants = [(f"{args.param}={v}", {**base, args.param: v}) for v in args.values.split(",")]
  for d in ds:
    print(f"== {d['route']}  (plant-simulated; desired curvature held to the log)")
    for label, ov in variants:
      ang, des, _, _ = simulate(d, plant, ov)
      print_metrics(label, metrics(d, ang, des))


if __name__ == "__main__":
  main()
