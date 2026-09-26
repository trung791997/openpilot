#!/usr/bin/env python3
"""Offline study: could a lateral-acceleration-space torque controller replace the angle PID?

Two questions, answered from rlogs only (offline log analysis, not a controller test):

  A. Kinematics. Measured curvature (calibrated livePose yaw rate / vEgo) against curvature
     predicted from the published wheel angle by the steady-state bicycle model

         kappa = theta_lin / (SR * L * (1 + K v^2))  -  g * K * roll / (1 + K v^2)

     which is VehicleModel.calc_curvature with K = -slip_factor. Three angle maps:
       M0  constant SR                                      (fit SR, K)
       M1  firmware VGR position map (steer_ratio.py), then constant SR   (fit SR, K)
       M2  the fork's road SR curve NRDR_SR_CURVE_BY_FP, times a scale    (fit scale, K)

  B. Torque model. Upstream latcontrol_torque feedforward inverted:

         torque = lat_accel / latAccelFactor + friction * s(lat_jerk) + offset

     with s() the same clipped-linear friction shape get_friction() uses. Torque is the value
     torqued learns from (-carOutput.actuatorsOutput.torque, shifted by liveDelay); lat accel
     is torqued's own (v * calibrated yaw - g sin(roll)) or the M0/M1 angle-derived one.

Usage:
  python tools/lateral/lat_accel_space_study.py /home/ubuntu/routes/00000263--b8afdda0eb [...]
"""
import argparse
import glob
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

G = 9.81
MPH = 0.44704
HZ = 20.0

FIELDS = [
  "t", "v", "a_ego", "angle", "rate", "pressed", "lblink", "rblink", "lcs", "torque_eps", "torque_driver", "yaw_cs",
  "lp_sr", "lp_sf", "lp_off", "lp_roll", "lp_valid", "lat_active", "cc_torque", "co_torque",
  "yaw_dev_z", "yaw_calib", "pose_roll", "calib_valid", "lag",
  "ltp_laf", "ltp_fric", "ltp_off", "ltp_valid", "ltp_points",
]

ANGLE_BINS = [(0, 5), (5, 15), (15, 45), (45, 90), (90, 1e9)]
SPEED_BANDS = [(0, 25), (25, 50), (50, 1e9)]  # mph
FRICTION_THRESHOLD = 0.3  # m/s^3, lat-jerk width of the friction ramp (latcontrol_torque default is 0.3)


# ----------------------------------------------------------------------------------------------
# extraction

def _segments(route_dir):
  segs = glob.glob(os.path.join(route_dir, "*", "rlog.zst")) + glob.glob(os.path.join(route_dir, "*", "rlog.bz2"))
  return sorted(segs, key=lambda p: int(os.path.basename(os.path.dirname(p))))


def extract_segment(seg):
  """One row per livePose (20 Hz) with the latest value of every other service."""
  from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator
  from openpilot.tools.lib.logreader import LogReader

  calib = PoseCalibrator()
  cp_bytes = None
  cs = lp = None
  lat_active = cc_torque = co_torque = 0.0
  lcs = 0.0
  lag = math.nan
  ltp = (math.nan, math.nan, math.nan, 0.0, 0.0)
  have_calib = False
  rows = []
  try:
    for m in LogReader(seg):
      w = m.which()
      if w == "carParams" and cp_bytes is None:
        cp_bytes = m.carParams.as_builder().to_bytes()
      elif w == "carState":
        cs = m.carState
      elif w == "liveParameters":
        lp = m.liveParameters
      elif w == "carControl":
        lat_active = float(m.carControl.latActive)
        cc_torque = m.carControl.actuators.torque
      elif w == "carOutput":
        co_torque = m.carOutput.actuatorsOutput.torque
      elif w == "modelV2":
        lcs = float(m.modelV2.meta.laneChangeState.raw)
      elif w == "liveCalibration":
        calib.feed_live_calib(m.liveCalibration)
        have_calib = True
      elif w == "liveDelay":
        lag = m.liveDelay.lateralDelay
      elif w == "liveTorqueParameters":
        q = m.liveTorqueParameters
        ltp = (q.latAccelFactorFiltered, q.frictionCoefficientFiltered, q.latAccelOffsetFiltered,
               float(q.liveValid), float(q.totalBucketPoints))
      elif w == "livePose" and cs is not None and lp is not None:
        pose = Pose.from_live_pose(m.livePose)
        cal = calib.build_calibrated_pose(pose)
        rows.append((
          m.logMonoTime * 1e-9, cs.vEgo, cs.aEgo, cs.steeringAngleDeg, cs.steeringRateDeg, float(cs.steeringPressed),
          float(cs.leftBlinker), float(cs.rightBlinker), lcs, cs.steeringTorqueEps, cs.steeringTorque, cs.yawRate,
          lp.steerRatio, lp.stiffnessFactor, lp.angleOffsetDeg, lp.roll, float(lp.valid), lat_active, cc_torque, co_torque,
          pose.angular_velocity.z, cal.angular_velocity.yaw, pose.orientation.roll,
          float(have_calib and calib.calib_valid), lag, *ltp,
        ))
  except Exception as e:  # a truncated final segment is common; keep what was read
    print(f"warning: {seg}: {e}", file=sys.stderr)
  return seg, np.array(rows, dtype=np.float64).reshape(-1, len(FIELDS)), cp_bytes


def extract_routes(route_dirs, cache_dir="/tmp/latstudy", procs=4):
  """Return {route: data dict}; per-route arrays cached as npz."""
  os.makedirs(cache_dir, exist_ok=True)
  out, todo = {}, []
  for rd in route_dirs:
    name = os.path.basename(os.path.normpath(rd))
    path = os.path.join(cache_dir, name + ".npz")
    if os.path.exists(path):
      z = np.load(path)
      out[name] = {k: z[k] for k in FIELDS} | {"cp_bytes": z["cp_bytes"].tobytes(), "route": name}
    else:
      todo += [(name, s) for s in _segments(rd)]
  if todo:
    with Pool(procs) as pool:
      res = pool.map(extract_segment, [s for _, s in todo], chunksize=1)
    by_route = {}
    for (name, _), (_, arr, cpb) in zip(todo, res, strict=True):
      r = by_route.setdefault(name, {"arrs": [], "cp": None})
      r["arrs"].append(arr)
      r["cp"] = r["cp"] or cpb
    for name, r in by_route.items():
      arr = np.concatenate(r["arrs"])
      if r["cp"] is None:
        print(f"warning: {name}: no carParams, skipped", file=sys.stderr)
        continue
      d = {k: arr[:, i] for i, k in enumerate(FIELDS)}
      np.savez_compressed(os.path.join(cache_dir, name + ".npz"), **d, cp_bytes=np.frombuffer(r["cp"], dtype=np.uint8))
      out[name] = d | {"cp_bytes": r["cp"], "route": name}
  return out


def car_info(cp_bytes):
  from cereal import car
  from opendbc.car.honda.steer_ratio import get_honda_vgr_inverse, get_honda_vgr_profile
  from openpilot.selfdrive.controls.lib.latcontrol_pid import NRDR_SR_CURVE_BY_FP
  with car.CarParams.from_bytes(cp_bytes) as CP:
    sf = CP.mass * (CP.tireStiffnessFront * CP.centerToFront - CP.tireStiffnessRear * (CP.wheelbase - CP.centerToFront)) / \
      (CP.wheelbase ** 2 * CP.tireStiffnessFront * CP.tireStiffnessRear)
    return {
      "fingerprint": str(CP.carFingerprint), "wheelbase": CP.wheelbase, "cp_sr": CP.steerRatio, "cp_k": -sf,
      "vgr_profile": get_honda_vgr_profile(CP.carFw), "vgr_inverse": get_honda_vgr_inverse(CP.flags),
      "sr_curve": NRDR_SR_CURVE_BY_FP.get(str(CP.carFingerprint)), "lateral_tuning": CP.lateralTuning.which(),
      "eps_fw": [bytes(f.fwVersion).split(b"\0")[0].decode(errors="ignore") for f in CP.carFw if f.ecu == "eps"],
    }


# ----------------------------------------------------------------------------------------------
# curvature models

def vgr_physical_to_linear_np(theta_deg, inverse):
  """Vectorised steer_ratio.vgr_physical_to_linear (published angle -> centre-equivalent angle)."""
  if inverse is None:
    return np.asarray(theta_deg, dtype=float)
  linear_bp, angle_bp = inverse
  return np.sign(theta_deg) * np.interp(np.abs(theta_deg), angle_bp, linear_bp)


def sr_curve_eff_angle(theta_deg, curve):
  """Angle divided by the curve's relative ratio, so constant SR = curve(0) * scale gives the curve."""
  bp, v = curve
  return np.asarray(theta_deg) * v[0] / np.interp(np.abs(theta_deg), bp, v)


def _curv_solve_for_k(x_deg, v, roll, kappa, wheelbase, k):
  den = 1.0 + k * v ** 2
  x = np.radians(x_deg) / (wheelbase * den)
  r = -G * k * roll / den
  y = kappa - r
  inv_sr = float(np.dot(x, y) / max(np.dot(x, x), 1e-12))
  res = y - inv_sr * x
  return inv_sr, float(np.mean(res ** 2))


def fit_bicycle(x_deg, v, roll, kappa, wheelbase, k_range=(-0.002, 0.012)):
  """Least-squares (SR, K) for kappa = x/(SR L (1+K v^2)) - g K roll/(1+K v^2). x = model angle, deg.

  1/SR is linear for fixed K, so K is found by grid + golden refinement on the profile cost.
  """
  ks = np.linspace(k_range[0], k_range[1], 71)
  costs = [_curv_solve_for_k(x_deg, v, roll, kappa, wheelbase, k)[1] for k in ks]
  i = int(np.argmin(costs))
  lo, hi = ks[max(i - 1, 0)], ks[min(i + 1, len(ks) - 1)]
  gr = (math.sqrt(5) - 1) / 2
  for _ in range(40):
    a, b = hi - gr * (hi - lo), lo + gr * (hi - lo)
    if _curv_solve_for_k(x_deg, v, roll, kappa, wheelbase, a)[1] < _curv_solve_for_k(x_deg, v, roll, kappa, wheelbase, b)[1]:
      hi = b
    else:
      lo = a
  k = 0.5 * (lo + hi)
  inv_sr, _ = _curv_solve_for_k(x_deg, v, roll, kappa, wheelbase, k)
  return 1.0 / inv_sr, k


def predict_curvature(x_deg, v, roll, wheelbase, sr, k):
  den = 1.0 + k * v ** 2
  return np.radians(x_deg) / (sr * wheelbase * den) - G * k * roll / den


def model_angles(d, info):
  """Model-input angle (deg) for M0/M1/M2, offset removed in the published coordinate."""
  theta = d["angle"] - d["lp_off"]
  out = {"M0": theta, "M1": vgr_physical_to_linear_np(theta, info["vgr_inverse"])}
  if info["sr_curve"] is not None:
    out["M2"] = sr_curve_eff_angle(theta, info["sr_curve"])
  return out


# ----------------------------------------------------------------------------------------------
# masks and derived signals

def _dilate(mask, n):
  if n <= 0:
    return mask
  return np.convolve(mask.astype(float), np.ones(2 * n + 1), mode="same") > 0


def derived(d, yaw_sign, exclude_pressed=True):
  v = d["v"]
  kappa = yaw_sign * d["yaw_calib"] / np.maximum(v, 0.1)
  base = (v > 4.0) & (d["calib_valid"] > 0) & np.isfinite(kappa)
  if exclude_pressed:
    base &= ~_dilate(d["pressed"] > 0, int(HZ))
  base &= (d["lblink"] == 0) & (d["rblink"] == 0) & (d["lcs"] == 0)
  # contiguous-time guard for derivatives
  dt = np.diff(d["t"], prepend=d["t"][0])
  base &= (dt < 0.2)
  return kappa, base


def empirical_yaw_sign(d):
  """+1 if calibrated yaw grows with left (positive) steering angle, else -1."""
  m = (d["v"] > 8) & (np.abs(d["angle"]) > 5) & (np.abs(d["angle"]) < 90) & (d["calib_valid"] > 0)
  if m.sum() < 100:
    return -1.0, math.nan
  c = np.corrcoef(d["angle"][m], d["yaw_calib"][m])[0, 1]
  return (1.0 if c > 0 else -1.0), c


def lagged(d, key, lag_s):
  """Value of key at t - lag (the torque that produced the motion now)."""
  return np.interp(d["t"] - lag_s, d["t"], d[key])


def smooth_rate(x, t, tau=0.25):
  n = max(1, int(tau * HZ))
  k = np.ones(n) / n
  xs = np.convolve(x, k, mode="same")
  return np.gradient(xs, t)


# ----------------------------------------------------------------------------------------------
# torque model

def friction_shape(jerk, threshold=FRICTION_THRESHOLD):
  return np.clip(jerk / threshold, -1.0, 1.0)


def fit_torque(la, jerk, torque):
  """OLS torque = la/LAF + friction * s(jerk) + offset. Returns (LAF, friction, offset, r2, rms)."""
  X = np.column_stack([la, friction_shape(jerk), np.ones_like(la)])
  coef, *_ = np.linalg.lstsq(X, torque, rcond=None)
  res = torque - X @ coef
  r2 = 1.0 - np.var(res) / max(np.var(torque), 1e-12)
  laf = 1.0 / coef[0] if abs(coef[0]) > 1e-9 else math.inf
  return laf, float(coef[1]), float(coef[2]), float(r2), float(np.sqrt(np.mean(res ** 2)))


def torque_residual_with(la, jerk, torque, laf, fric, off):
  res = torque - (la / laf + fric * friction_shape(jerk) + off)
  return float(np.sqrt(np.mean(res ** 2)))


# ----------------------------------------------------------------------------------------------
# reporting

def _rms(x):
  return float(np.sqrt(np.mean(x ** 2))) if len(x) else math.nan


def study_kinematics(routes, infos, yaw_signs, exclude_pressed=True):
  print("\n=== A. kinematics: curvature from wheel angle vs measured yaw (driver-pressed frames " +
        f"{'excluded' if exclude_pressed else 'INCLUDED'}) ===")
  pooled = {"v": [], "roll": [], "kappa": [], "ang": [], "route": [], "x": {}}
  print(f"{'route':22s} {'n':>6s} {'paramsdSR':>9s} | {'M0 SR':>6s} {'K':>7s} | {'M1 SR':>6s} {'K':>7s} | {'M2 sc':>6s} {'K':>7s}" +
        " | rmsLA M0/M1/M2 (m/s^2)")
  per_route = {}
  for name, d in routes.items():
    info = infos[name]
    kappa, base = derived(d, yaw_signs[name], exclude_pressed)
    m = base & (np.abs(d["rate"]) < 20) & (d["lp_valid"] > 0)
    angs = model_angles(d, info)
    v, roll, L = d["v"][m], d["lp_roll"][m], info["wheelbase"]
    fits, rms = {}, {}
    for key, x in angs.items():
      sr, k = fit_bicycle(x[m], v, roll, kappa[m], L)
      fits[key] = (sr, k)
      rms[key] = _rms((kappa[m] - predict_curvature(x[m], v, roll, L, sr, k)) * v ** 2)
      pooled["x"].setdefault(key, []).append(x[m])
    per_route[name] = fits
    pooled["v"].append(v)
    pooled["roll"].append(roll)
    pooled["kappa"].append(kappa[m])
    pooled["ang"].append(np.abs(d["angle"][m]))
    lp_sr = np.nanmedian(d["lp_sr"][d["lp_valid"] > 0]) if np.any(d["lp_valid"] > 0) else math.nan
    m2 = fits.get("M2", (math.nan, math.nan))
    m2_scale = m2[0] / info["sr_curve"][1][0] if info["sr_curve"] is not None else math.nan
    print(f"{name:22s} {m.sum():6d} {lp_sr:9.2f} | {fits['M0'][0]:6.2f} {fits['M0'][1]:7.4f} | {fits['M1'][0]:6.2f} " +
          f"{fits['M1'][1]:7.4f} | {m2_scale:6.3f} {m2[1]:7.4f} | "
          + "/".join(f"{rms[k]:.3f}" for k in angs))
  for key in pooled["x"]:
    srs = [f[key][0] for f in per_route.values()]
    print(f"per-route {key} SR spread: min {min(srs):.2f} max {max(srs):.2f} std {np.std(srs):.2f}")

  v = np.concatenate(pooled["v"])
  roll = np.concatenate(pooled["roll"])
  kappa = np.concatenate(pooled["kappa"])
  ang = np.concatenate(pooled["ang"])
  L = next(iter(infos.values()))["wheelbase"]
  preds = {}
  print("\npooled fit (one parameter set for all routes):")
  for key, xs in pooled["x"].items():
    x = np.concatenate(xs)
    sr, k = fit_bicycle(x, v, roll, kappa, L)
    preds[key] = predict_curvature(x, v, roll, L, sr, k)
    print(f"  {key}: SR {sr:.3f}  K {k:.5f} s^2/m^2  (CP K {next(iter(infos.values()))['cp_k']:.5f})")
  # per-route params, evaluated per route (route-fitted) are shown above; bins use the pooled fit
  print("\npooled-fit lat-accel residual rms (m/s^2) [and bias], by |wheel angle| x speed band:")
  hdr = f"{'|angle| deg':12s} {'mph':8s} {'n':>6s} " + " ".join(f"{k:>14s}" for k in preds)
  print(hdr)
  vm = v / MPH
  for a0, a1 in ANGLE_BINS + [(0, 1e9)]:
    for s0, s1 in SPEED_BANDS + [(0, 1e9)]:
      b = (ang >= a0) & (ang < a1) & (vm >= s0) & (vm < s1)
      if b.sum() < 20:
        continue
      cells = []
      for key in preds:
        r = (kappa[b] - preds[key][b]) * v[b] ** 2
        cells.append(f"{_rms(r):7.3f} {np.mean(r):+6.3f}")
      a_lbl = f"{a0:g}-{a1:g}" if a1 < 1e8 else (f"{a0:g}+" if a0 > 0 else "all")
      s_lbl = f"{s0:g}-{s1:g}" if s1 < 1e8 else (f"{s0:g}+" if s0 > 0 else "all")
      print(f"{a_lbl:12s} {s_lbl:8s} {b.sum():6d} " + " ".join(f"{c:>14s}" for c in cells))
  # effective ratio by angle bin, per speed band: theta / (kappa L (1+Kv^2)) using pooled M0 K
  return per_route


def study_torque(routes, infos, yaw_signs, kin_fits, torque_key):
  print(f"\n=== B. torque model ({torque_key}) ===")
  cols = {"la_meas": [], "la_M0": [], "la_M1": [], "jerk": [], "tq": [], "v": []}
  for name, d in routes.items():
    info = infos[name]
    kappa, base = derived(d, yaw_signs[name])
    lag = float(np.nanmedian(d["lag"])) if np.any(np.isfinite(d["lag"])) else 0.2
    if torque_key in ("co_torque", "cc_torque"):
      tq = -lagged(d, torque_key, lag)  # torqued's sign and lag
    else:
      tq = lagged(d, torque_key, lag)
    active = _dilate(d["lat_active"] < 0.5, int(HZ)) == 0  # lat active for +-1 s
    m = base & active & (d["lp_valid"] > 0)
    # torqued convention: v * calibrated yaw - g sin(pose roll)
    la_meas = d["v"] * d["yaw_calib"] - np.sin(d["pose_roll"]) * G
    angs = model_angles(d, info)
    la = {}
    for key in ("M0", "M1"):
      sr, k = kin_fits[name][key]
      # steering-induced curvature only; gravity term handled like torqued via pose roll
      la[key] = yaw_signs[name] * predict_curvature(angs[key], d["v"], 0.0, info["wheelbase"], sr, k) * d["v"] ** 2 \
        - np.sin(d["pose_roll"]) * G
    jerk = smooth_rate(la["M0"], d["t"])
    for k_, arr in (("la_meas", la_meas), ("la_M0", la["M0"]), ("la_M1", la["M1"]), ("jerk", jerk), ("tq", tq), ("v", d["v"])):
      cols[k_].append(arr[m])
  c = {k: np.concatenate(v) for k, v in cols.items()}
  corr = np.corrcoef(c["la_meas"], c["tq"])[0, 1]
  print(f"frames {len(c['tq'])}; corr(torque, la_meas) {corr:+.3f}")
  sign = 1.0 if corr > 0 else -1.0
  c["tq"] = sign * c["tq"]
  if sign < 0:
    print("  (torque sign flipped so positive torque produces positive lat accel)")
  vm = c["v"] / MPH
  bands = [(s0, s1, f"{s0:g}-{s1:g}" if s1 < 1e8 else f"{s0:g}+") for s0, s1 in SPEED_BANDS]
  fine = [(s, s + 5, f"{s}-{s + 5}") for s in range(10, 80, 5)]
  for la_key in ("la_meas", "la_M0", "la_M1"):
    la = c[la_key]
    keep = np.abs(la) < 3.0
    laf, fr, off, r2, rms = fit_torque(la[keep], c["jerk"][keep], c["tq"][keep])
    print(f"\n[{la_key}] pooled: LAF {laf:.3f} m/s^2/unit  friction {fr:.4f}  offset {off:+.4f}  R2 {r2:.3f}  rms {rms:.4f}")
    print(f"  {'band mph':9s} {'n':>6s} {'LAF':>7s} {'fric':>7s} {'off':>7s} {'R2':>6s} {'rms_band':>8s} {'rms_pooled':>10s}")
    for label, bins in (("coarse", bands), ("5 mph", fine)):
      lafs = []
      for s0, s1, lbl in bins:
        b = keep & (vm >= s0) & (vm < s1)
        if b.sum() < 200:
          continue
        f = fit_torque(la[b], c["jerk"][b], c["tq"][b])
        pr = torque_residual_with(la[b], c["jerk"][b], c["tq"][b], laf, fr, off)
        lafs.append(f[0])
        print(f"  {lbl:9s} {b.sum():6d} {f[0]:7.3f} {f[1]:7.4f} {f[2]:+7.4f} {f[3]:6.3f} {f[4]:8.4f} {pr:10.4f}")
      if lafs:
        print(f"  {label} LAF max/min ratio {max(lafs) / min(lafs):.2f}")
  # linearity
  print("\nlinearity: mean torque by la_meas bin (m/s^2), per coarse speed band")
  edges = np.array([-3, -2, -1.5, -1, -0.6, -0.3, -0.1, 0.1, 0.3, 0.6, 1, 1.5, 2, 3])
  print(f"  {'la bin':>12s} " + " ".join(f"{lbl:>14s}" for _, _, lbl in bands))
  for lo, hi in zip(edges[:-1], edges[1:], strict=True):
    cells = []
    for s0, s1, _ in bands:
      b = (c["la_meas"] >= lo) & (c["la_meas"] < hi) & (vm >= s0) & (vm < s1)
      cells.append(f"{np.mean(c['tq'][b]):+7.3f} ({b.sum():5d})" if b.sum() >= 20 else f"{'-':>14s}")
    print(f"  {lo:+5.1f}..{hi:+4.1f} " + " ".join(f"{x:>14s}" for x in cells))


def report_live_torque(routes):
  print("\n=== liveTorqueParameters (torqued on device) ===")
  for name, d in routes.items():
    laf, fr, valid, pts = d["ltp_laf"], d["ltp_fric"], d["ltp_valid"], d["ltp_points"]
    ok = np.isfinite(laf)
    if not ok.any():
      print(f"{name}: not logged")
      continue
    print(f"{name}: LAF filt start {laf[ok][0]:.3f} end {laf[ok][-1]:.3f} median {np.median(laf[ok]):.3f}; " +
          f"friction filt end {fr[ok][-1]:.4f}; liveValid {100 * np.mean(valid[ok]):.0f}% of samples; " +
          f"points end {pts[ok][-1]:.0f}")


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+", help="route directories containing <seg>/rlog.zst")
  ap.add_argument("--cache-dir", default="/tmp/latstudy")
  ap.add_argument("--procs", type=int, default=4)
  ap.add_argument("--kin-include-pressed", action="store_true",
                  help="also run study A with driver-pressed frames kept (more large-angle samples)")
  args = ap.parse_args(argv)

  routes = extract_routes(args.routes, args.cache_dir, args.procs)
  infos, yaw_signs = {}, {}
  print("=== routes ===")
  for name, d in routes.items():
    infos[name] = info = car_info(d["cp_bytes"])
    yaw_signs[name], corr = empirical_yaw_sign(d)
    yc = d["yaw_cs"]
    print(f"{name}: {info['fingerprint']} eps_fw {info['eps_fw']} vgr {info['vgr_profile']} tuning {info['lateral_tuning']} " +
          f"frames {len(d['t'])} corr(angle, calib yaw) {corr:+.3f} carState.yawRate nonzero {np.mean(yc != 0) * 100:.0f}% " +
          f"corr(dev z, calib yaw) {np.corrcoef(d['yaw_dev_z'], d['yaw_calib'])[0, 1]:+.3f} " +
          f"lag median {np.nanmedian(d['lag']):.3f}s")
  kin = study_kinematics(routes, infos, yaw_signs)
  if args.kin_include_pressed:
    study_kinematics(routes, infos, yaw_signs, exclude_pressed=False)
  study_torque(routes, infos, yaw_signs, kin, "co_torque")
  study_torque(routes, infos, yaw_signs, kin, "cc_torque")
  if any(np.any(d["torque_eps"] != 0) for d in routes.values()):
    study_torque(routes, infos, yaw_signs, kin, "torque_eps")
  else:
    print("\ncarState.steeringTorqueEps is 0 on every frame (not populated by the Honda port); EPS view skipped")
  report_live_torque(routes)


if __name__ == "__main__":
  main()
