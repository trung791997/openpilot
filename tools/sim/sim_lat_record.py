#!/usr/bin/env python3
"""Records lateral control variables to lat_pid_sim.npz and writes episode metadata."""
from __future__ import annotations

import argparse
import json
import os
import time
from typing import Any

import numpy as np

from openpilot.tools.lateral.lat_pid_sim import FIELDS, TUNING_KEYS


def _get(obj: Any, key: str, default: Any = None) -> Any:
  if obj is None:
    return default
  if isinstance(obj, dict):
    return obj.get(key, default)
  if hasattr(obj, "__getitem__"):
    try:
      return obj[key]
    except (KeyError, IndexError, TypeError):
      pass
  return getattr(obj, key, default)


def build_row(sm_like: Any, t0: float | None = None) -> list[float]:
  c = _get(sm_like, "controlsState")
  cs = _get(sm_like, "carState")
  cc = _get(sm_like, "carControl")
  co = _get(sm_like, "carOutput")
  lp = _get(sm_like, "liveParameters")

  if hasattr(sm_like, "t"):
    t = float(sm_like.t)
  elif isinstance(sm_like, dict) and "t" in sm_like:
    t = float(sm_like["t"])
  else:
    lmt = _get(sm_like, "logMonoTime")
    if isinstance(lmt, dict):
      raw_lmt = lmt.get("controlsState", 0)
    elif lmt is not None:
      raw_lmt = lmt
    else:
      raw_lmt = _get(c, "logMonoTime", 0)

    raw_val = float(raw_lmt) if raw_lmt is not None else 0.0
    if t0 is not None:
      diff = raw_val - t0
      t = diff * 1e-9 if abs(diff) > 1e6 else diff
    else:
      t = raw_val * 1e-9 if abs(raw_val) > 1e6 else raw_val

  v = float(_get(cs, "vEgo", 0.0))
  a_ego = float(_get(cs, "aEgo", 0.0))
  angle = float(_get(cs, "steeringAngleDeg", 0.0))
  rate = float(_get(cs, "steeringRateDeg", 0.0))
  pressed = float(bool(_get(cs, "steeringPressed", False)))
  eps_torque = float(_get(cs, "steeringTorque", 0.0))
  lblink = float(bool(_get(cs, "leftBlinker", False)))
  rblink = float(bool(_get(cs, "rightBlinker", False)))

  sr = float(_get(lp, "steerRatio", 0.0))
  stiff = float(_get(lp, "stiffnessFactor", 0.0))
  offset = float(_get(lp, "angleOffsetDeg", 0.0))
  roll = float(_get(lp, "roll", 0.0))

  des_curv = float(_get(c, "desiredCurvature", 0.0))
  active = float(bool(_get(cc, "latActive", False)))

  lcs = _get(c, "lateralControlState")
  if lcs is not None and hasattr(lcs, "which"):
    w = lcs.which() if callable(lcs.which) else lcs.which
    p = _get(lcs, "pidState", lcs) if w == "pidState" else None
  else:
    p = _get(lcs, "pidState", lcs) if lcs is not None else None
  if p is None and c is not None:
    p = _get(c, "pidState")

  out = float(_get(p, "output", 0.0))
  des_angle = float(_get(p, "steeringAngleDesiredDeg", 0.0))
  log_p = float(_get(p, "p", 0.0))
  log_i = float(_get(p, "i", 0.0))
  log_f = float(_get(p, "f", 0.0))

  act = _get(cc, "actuators")
  cc_torque = float(_get(act, "torque", 0.0)) if act is not None else 0.0

  act_out = _get(co, "actuatorsOutput")
  co_torque = float(_get(act_out, "torque", 0.0)) if act_out is not None else 0.0

  lane_change = float(_get(sm_like, "lane_change", 0.0))

  return [
    t, v, a_ego, angle, rate, pressed, eps_torque, lblink, rblink,
    sr, stiff, offset, roll, des_curv, active, out, des_angle,
    cc_torque, co_torque, lane_change, log_p, log_i, log_f,
  ]


def write_npz(outdir: str, rows: list | np.ndarray, cp_bytes: bytes | np.ndarray, params: dict | str) -> None:
  os.makedirs(outdir, exist_ok=True)
  arr = np.array(rows, dtype=np.float64)
  if arr.ndim == 1:
    arr = arr.reshape(1, -1) if len(rows) > 0 else np.zeros((0, len(FIELDS)), dtype=np.float64)
  elif arr.size == 0:
    arr = np.zeros((0, len(FIELDS)), dtype=np.float64)

  data = {k: arr[:, i] for i, k in enumerate(FIELDS)}
  if isinstance(cp_bytes, (bytes, bytearray, memoryview)):
    cp_arr = np.frombuffer(cp_bytes, dtype=np.uint8)
  elif isinstance(cp_bytes, np.ndarray):
    cp_arr = cp_bytes.astype(np.uint8)
  else:
    cp_arr = np.frombuffer(bytes(cp_bytes), dtype=np.uint8)

  if isinstance(params, (dict, list)):
    params_str = json.dumps(params)
  else:
    params_str = str(params)

  cache_path = os.path.join(outdir, "lat_pid_sim.npz")
  np.savez_compressed(
    cache_path,
    **data,
    cp_bytes=cp_arr,
    params=params_str,
  )


def check_offroad(bridge_log_path: str | None) -> bool:
  if not bridge_log_path or not os.path.exists(bridge_log_path):
    return False
  with open(bridge_log_path, encoding="utf-8", errors="ignore") as f:
    for line in f:
      line_lower = line.lower()
      if "out_of_road" in line_lower or "out_of_lane" in line_lower or "crash" in line_lower:
        return True
  return False


def write_episode(outdir: str, seconds: float, rows: list | np.ndarray | int,
                  engaged_s: float | None = None, bridge_log: str | None = None) -> dict[str, Any]:
  os.makedirs(outdir, exist_ok=True)
  if isinstance(rows, (list, tuple, np.ndarray)):
    num_rows = len(rows)
    if engaged_s is None:
      if num_rows > 0:
        arr = np.asarray(rows)
        active_idx = FIELDS.index("active")
        # per-row dt from the actual recording rate (seconds / rows) rather than an assumed
        # 100 Hz, so this stays correct if controlsState publishes at a different rate.
        dt = float(seconds) / num_rows
        engaged_s = float(np.sum(arr[:, active_idx] > 0.5) * dt)
      else:
        engaged_s = 0.0
  else:
    num_rows = int(rows)
    if engaged_s is None:
      engaged_s = 0.0

  offroad = check_offroad(bridge_log)
  data = {
    "seconds": float(seconds),
    "rows": int(num_rows),
    "engaged_s": float(engaged_s),
    "offroad": bool(offroad),
  }
  ep_path = os.path.join(outdir, "episode.json")
  with open(ep_path, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2)
  return data


def record(outdir: str, secs: float, bridge_log: str | None = None) -> None:
  import cereal.messaging as messaging
  from openpilot.common.params import Params

  sm = messaging.SubMaster([
    "controlsState",
    "carState",
    "carControl",
    "carOutput",
    "liveParameters",
    "carParams",
    "modelV2",
    "starpilotLateralState",
    "liveCalibration",
    "starpilotPlan",
  ], poll="controlsState")

  rows = []
  lanes = []  # per row: modelV2 laneLineProbs (4) + roadEdgeStds (2), saved to lanes.npz (perception of the sim's roads)
  model_rt = []  # per row: modelV2 frameDropPerc, modelExecutionTime (s): whether modeld keeps up on this machine
  eps_ff = []  # per row: starpilotLateralState epsFfWeight, epsFfFeedforward (pidState.f under the PID is the raw kf term)
  calib = []  # per row: liveCalibration calStatus, calPerc, validBlocks, rpyCalib (3); saved to lanes.npz calib (N x 6), the live
  #             calibration modeld warps with, which is not what CalibrationParams holds at episode end
  plan = []  # per row: modelV2 action.desiredCurvature (the model's request before controlsd's clip_curvature; compare with
  #            des_curv to see the ISO lateral-accel clip, which controlsState does not publish), starpilotPlan
  #            cscControllingSpeed, cscSpeed, vCruise (m/s); saved to lanes.npz plan (N x 4)
  t_mono = []  # per row: host time.monotonic() at the row, the clock metadrive_process writes into frames/lane_gt.csv t_mono, so
  # the two files align exactly. Before 2026-09-27 they shared no clock and the npz begins ~16 s after the world starts
  # (the car is already at 7-8 m/s), so distance-from-npz-start windows landed 30-38 m off the map (varying per run).
  t0 = None
  start_mono = time.monotonic()
  cp_bytes = None

  while time.monotonic() - start_mono < secs:
    sm.update(100)
    if not sm.updated["controlsState"]:
      continue

    if not sm.seen["carState"] or not sm.seen["liveParameters"]:
      continue

    cs = sm["controlsState"]
    lcs = getattr(cs, "lateralControlState", None)
    if lcs is not None and hasattr(lcs, "which") and lcs.which() != "pidState":
      continue

    if sm.seen["carParams"] and cp_bytes is None:
      cp_msg = sm["carParams"]
      if hasattr(cp_msg, "as_builder"):
        cp_bytes = cp_msg.as_builder().to_bytes()
      elif hasattr(cp_msg, "to_bytes"):
        cp_bytes = cp_msg.to_bytes()
      else:
        cp_bytes = bytes(cp_msg)

    if t0 is None:
      t0 = sm.logMonoTime["controlsState"]

    row = build_row(sm, t0=t0)
    rows.append(row)
    mv = sm["modelV2"]
    lanes.append(list(mv.laneLineProbs)[:4] + list(mv.roadEdgeStds)[:2] if sm.seen["modelV2"] else [np.nan] * 6)
    model_rt.append([mv.frameDropPerc, mv.modelExecutionTime] if sm.seen["modelV2"] else [np.nan] * 2)
    sl = sm["starpilotLateralState"]
    eps_ff.append([sl.epsFfWeight, sl.epsFfFeedforward] if sm.seen["starpilotLateralState"] else [np.nan] * 2)
    lc = sm["liveCalibration"]
    calib.append([float(lc.calStatus.raw), lc.calPerc, lc.validBlocks] + (list(lc.rpyCalib) + [np.nan] * 3)[:3]
                 if sm.seen["liveCalibration"] else [np.nan] * 6)
    sp = sm["starpilotPlan"]
    plan.append([mv.action.desiredCurvature if sm.seen["modelV2"] else np.nan] +
                ([float(sp.cscControllingSpeed), sp.cscSpeed, sp.vCruise] if sm.seen["starpilotPlan"] else [np.nan] * 3))
    t_mono.append(time.monotonic())

  elapsed = time.monotonic() - start_mono
  lane_arr = np.array([(l + [np.nan] * 6)[:6] for l in lanes], dtype=np.float64).reshape(-1, 6)
  np.savez_compressed(os.path.join(outdir, "lanes.npz"), lane_probs=lane_arr[:, :4], edge_stds=lane_arr[:, 4:],
                      model_rt=np.array(model_rt, dtype=np.float64).reshape(-1, 2),
                      eps_ff=np.array(eps_ff, dtype=np.float64).reshape(-1, 2),
                      calib=np.array(calib, dtype=np.float64).reshape(-1, 6),
                      plan=np.array(plan, dtype=np.float64).reshape(-1, 4),
                      t_mono=np.array(t_mono, dtype=np.float64))
  if cp_bytes is None:
    if sm.seen["carParams"]:
      cp_bytes = sm["carParams"].as_builder().to_bytes()
    else:
      # an episode without carParams cannot be replayed (no car, no tune); fail loudly instead of writing an empty CP
      raise RuntimeError("no carParams received: is the sim up and controlsd running? (see the bridge log)")

  p = Params()
  params = {}
  for k in TUNING_KEYS:
    v = p.get(k)
    if v is not None:
      try:
        params[k] = v.decode() if isinstance(v, bytes) else str(v)
      except Exception:
        pass

  write_npz(outdir, rows, cp_bytes, params)
  write_episode(outdir, elapsed, rows, bridge_log=bridge_log)


def main() -> None:
  parser = argparse.ArgumentParser(description="Record lateral control variables to lat_pid_sim.npz")
  parser.add_argument("outdir", help="Output directory")
  parser.add_argument("secs", type=float, help="Duration to record in seconds")
  parser.add_argument("--bridge-log", default=None, help="Path to bridge log file")
  args = parser.parse_args()

  record(args.outdir, args.secs, bridge_log=args.bridge_log)


if __name__ == "__main__":
  main()
