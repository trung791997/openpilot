#!/usr/bin/env python3
"""Stacked time-series plots of longitudinal behaviour from local rlogs, for reviewing ICBM and
stock-ACC tuning after a drive.

  long_timeseries.py <route_dir> --window 120 160 [--out DIR]
  long_timeseries.py <route_dir> --bookmarks [--before 25 --after 10] [--out DIR]

<route_dir> holds one directory per segment, each with rlog.zst (or rlog/rlog.bz2).
Times are route seconds from the first message of segment 0, the same base as the icbm* scripts.

Panels, top to bottom:
  1. speed (mph): ego, cluster set speed, openpilot vCruise, lead speed, planner min speed
  2. lead distance (m): radarState.leadOne dRel coloured radar-fused vs vision-only, the model's lead x,
     and the nearest measured liveTracks point within |yRel| < 1.8 m (radar saw it vs openpilot used it)
  3. accel (m/s^2): aEgo, planner aTarget, ACC_CONTROL.ACCEL_COMMAND (stock ACC, or openpilot's own under
     alpha long, read from the panda TX echo)
  4. ICBM presses (sendcan 662), driver presses (can 662), gas and brake pedal; grey shading = engaged
Bookmarks draw as dashed lines on every panel.
"""
from __future__ import annotations

import argparse
import os
from collections import defaultdict
from pathlib import Path

import numpy as np

from openpilot.tools.lib.logreader import _LogFileReader

MS_TO_MPH = 2.23694
SCM_BUTTONS = 662  # CRUISE_BUTTONS = (dat[0] >> 5) & 7: 4 = RES_ACCEL, 3 = DECEL_SET, 2 = CANCEL
ACC_CONTROL = 0x1DF
IN_PATH_Y = 1.8  # m; raw |yRel| for the nearest-radar-point trace, no path curvature applied
PARAM_KEYS = ("RedneckCruise", "ICBMFarLead", "ICBMCounterSync", "AlphaLongitudinalEnabled", "ExperimentalMode", "BoschARailInterval")


def segment_logs(route_dir: Path) -> list[tuple[int, Path]]:
  out = []
  for d in route_dir.iterdir():
    if not d.is_dir() or not d.name.isdigit():
      continue
    for name in ("rlog.zst", "rlog", "rlog.bz2"):
      if (d / name).exists():
        out.append((int(d.name), d / name))
        break
  return sorted(out)


def _acc_parser():
  try:
    from opendbc.can.parser import CANParser
    return CANParser("honda_civic_hatchback_ex_2017_can_generated", [("ACC_CONTROL", 0)], 1)
  except Exception as e:  # the DBC or the compiled parser may be missing in a bare checkout
    print(f"ACC_CONTROL decode off: {e}")
    return None


def extract(route_dir: Path, t_min: float | None, t_max: float | None) -> dict:
  s = defaultdict(list)
  params: dict[str, str] = {}
  bookmarks: list[float] = []
  cp = _acc_parser()
  acc_seen = False
  T0 = None
  for _seg, path in segment_logs(route_dir):
    first = True
    for m in _LogFileReader(str(path)):
      if T0 is None:
        T0 = int(m.logMonoTime)
      t = (int(m.logMonoTime) - T0) / 1e9
      w = m.which()
      if first and w != "initData":
        first = False
        if t_max is not None and t > t_max + 1:
          break
      if w == "initData":
        for e in m.initData.params.entries:
          k = e.key.decode() if isinstance(e.key, bytes) else str(e.key)
          if k in PARAM_KEYS:
            v = e.value
            params[k] = v.decode(errors="replace") if isinstance(v, bytes) else str(v)
        continue
      if w in ("bookmarkButton", "userBookmark"):
        bookmarks.append(t)
        continue
      if t_max is not None and t > t_max:
        break
      if t_min is not None and t < t_min:
        continue
      if w == "carState":
        cs = m.carState
        s["cs_t"].append(t)
        s["vEgo"].append(cs.vEgo * MS_TO_MPH)
        s["aEgo"].append(cs.aEgo)
        s["setCluster"].append(cs.cruiseState.speedCluster * MS_TO_MPH if cs.cruiseState.speedCluster > 0 else np.nan)
        s["vCruise"].append(cs.vCruise / 1.609344 if cs.vCruise < 250 else np.nan)
        s["enabled"].append(cs.cruiseState.enabled)
        s["gas"].append(cs.gasPressed)
        s["brake"].append(cs.brakePressed)
      elif w == "radarState":
        l = m.radarState.leadOne
        s["rs_t"].append(t)
        s["dRel"].append(l.dRel if l.status else np.nan)
        s["vLead"].append(l.vLead * MS_TO_MPH if l.status else np.nan)
        s["radar"].append(bool(l.status and l.radar))
      elif w == "liveTracks":
        near = [pt.dRel for pt in m.liveTracks.points if pt.measured and abs(pt.yRel) < IN_PATH_Y]
        s["lt_t"].append(t)
        s["ltNear"].append(min(near) if near else np.nan)
      elif w == "longitudinalPlan":
        lp = m.longitudinalPlan
        s["lp_t"].append(t)
        s["aTarget"].append(lp.aTarget)
        s["planMin"].append(min(lp.speeds) * MS_TO_MPH if len(lp.speeds) else np.nan)
      elif w == "modelV2":
        leads = m.modelV2.leadsV3
        s["md_t"].append(t)
        s["mdLead"].append(leads[0].x[0] if len(leads) and leads[0].prob > 0.5 else np.nan)
      elif w == "sendcan":
        for f in m.sendcan:
          if f.address == SCM_BUTTONS:
            b = (bytes(f.dat)[0] >> 5) & 7
            if b:
              s["op_t"].append(t)
              s["op_b"].append(b)
      elif w == "can":
        frames = []
        for f in m.can:
          if f.address == SCM_BUTTONS and f.src in (0, 1) and len(f.dat) == 4:
            b = (bytes(f.dat)[0] >> 5) & 7
            if b:
              s["drv_t"].append(t)
              s["drv_b"].append(b)
          elif f.address == ACC_CONTROL and cp is not None:
            frames.append((f.address, bytes(f.dat), f.src & 0x7F))
        if frames:
          cp.update([(int(m.logMonoTime), frames)])
          s["acc_t"].append(t)
          s["accCmd"].append(cp.vl["ACC_CONTROL"]["ACCEL_COMMAND"])
          acc_seen = True
  out = {k: np.asarray(v) for k, v in s.items()}
  out["params"] = params
  out["bookmarks"] = bookmarks
  out["acc_seen"] = acc_seen
  return out


def _window(d: dict, key_t: str, keys: tuple[str, ...], t0: float, t1: float):
  t = d.get(key_t, np.array([]))
  m = (t >= t0) & (t <= t1)
  return t[m], [d.get(k, np.array([]))[m] for k in keys]


def _spans(t: np.ndarray, flag: np.ndarray):
  """Contiguous True runs of flag as (start, end) times."""
  if not len(t):
    return []
  f = flag.astype(bool)
  edges = np.flatnonzero(np.diff(f.astype(int)))
  starts = [0] + list(edges + 1)
  return [(t[a], t[min(b, len(t) - 1)]) for a, b in zip(starts, list(edges + 1) + [len(t)], strict=True) if f[a]]


def plot(d: dict, t0: float, t1: float, title: str, path: Path) -> None:
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt

  fig, ax = plt.subplots(4, 1, sharex=True, figsize=(14, 11), gridspec_kw={"height_ratios": [3, 2.2, 2.2, 1.3]})
  ct, (v, set_c, vc, en, gas, brk, a_ego) = _window(d, "cs_t", ("vEgo", "setCluster", "vCruise", "enabled", "gas", "brake", "aEgo"), t0, t1)
  rt, (dr, vl, rad) = _window(d, "rs_t", ("dRel", "vLead", "radar"), t0, t1)
  pt, (at, pmin) = _window(d, "lp_t", ("aTarget", "planMin"), t0, t1)
  mt, (ml,) = _window(d, "md_t", ("mdLead",), t0, t1)

  a = ax[0]
  a.plot(ct, v, "k", lw=1.6, label="ego")
  a.plot(ct, set_c, color="tab:orange", lw=1.6, drawstyle="steps-post", label="cluster set")
  a.plot(ct, vc, color="tab:orange", lw=1, ls=":", drawstyle="steps-post", label="openpilot vCruise")
  a.plot(rt, vl, color="tab:blue", lw=1.2, label="lead")
  a.plot(pt, pmin, color="tab:green", lw=1, alpha=0.8, label="plan min speed")
  a.set_ylabel("mph")
  a.legend(loc="upper left", ncol=5, fontsize=8)

  a = ax[1]
  a.plot(rt, np.where(rad, dr, np.nan), color="tab:red", lw=1.6, label="lead dRel (radar)")
  a.plot(rt, np.where(~rad, dr, np.nan), color="tab:purple", lw=1.6, label="lead dRel (vision only)")
  a.plot(mt, ml, color="gray", lw=0.8, alpha=0.7, label="model lead x (prob>0.5)")
  lt, (ln,) = _window(d, "lt_t", ("ltNear",), t0, t1)
  a.plot(lt, ln, ".", color="tab:red", ms=2, alpha=0.5, label=f"nearest radar point |y|<{IN_PATH_Y} m (raw)")
  a.set_ylabel("m")
  a.legend(loc="upper right", ncol=2, fontsize=8)

  a = ax[2]
  a.axhline(0, color="lightgray", lw=0.8)
  a.plot(ct, a_ego, "k", lw=1.2, label="aEgo")
  a.plot(pt, at, color="tab:green", lw=1.2, label="plan aTarget")
  if d["acc_seen"]:
    act, (acc,) = _window(d, "acc_t", ("accCmd",), t0, t1)
    who = "openpilot" if d["params"].get("AlphaLongitudinalEnabled") == "1" else "stock ACC"
    a.plot(act, acc, color="tab:red", lw=1, alpha=0.8, drawstyle="steps-post", label=f"{who} ACC_CONTROL accel")
  a.set_ylabel("m/s²")
  a.legend(loc="upper left", ncol=3, fontsize=8)

  a = ax[3]
  lanes = {"ICBM RES+": 5, "ICBM SET-": 4, "driver RES+": 3, "driver SET-": 2, "gas": 1, "brake": 0}
  ot, (ob,) = _window(d, "op_t", ("op_b",), t0, t1)
  dt, (db,) = _window(d, "drv_t", ("drv_b",), t0, t1)
  for tt, mask, y, c in ((ot, ob == 4, 5, "tab:green"), (ot, ob == 3, 4, "tab:red"), (dt, db == 4, 3, "tab:green"), (dt, db == 3, 2, "tab:red")):
    a.plot(tt[mask], np.full(mask.sum(), y), "|", color=c, ms=10)
  for flag, y, c in ((gas, 1, "tab:green"), (brk, 0, "tab:red")):
    a.broken_barh([(s0, max(s1 - s0, 0.05)) for s0, s1 in _spans(ct, flag)], (y - 0.3, 0.6), color=c, alpha=0.7)
  a.set_yticks(list(lanes.values()), list(lanes.keys()), fontsize=8)
  a.set_ylim(-0.6, 5.6)
  a.set_xlabel("route time (s)   grey shading = cruise engaged, magenta dashes = bookmarks")

  for s0, s1 in _spans(ct, en):
    for x in ax:
      x.axvspan(s0, s1, color="gray", alpha=0.07, lw=0)
  for b in d["bookmarks"]:
    if t0 <= b <= t1:
      for x in ax:
        x.axvline(b, color="magenta", ls="--", lw=1)
  for x in ax:
    x.grid(alpha=0.3)
  ax[-1].set_xlim(t0, t1)
  fig.suptitle(title, fontsize=10)
  fig.tight_layout()
  fig.savefig(path, dpi=110)
  plt.close(fig)


def main() -> None:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("route_dir", type=Path)
  g = p.add_mutually_exclusive_group(required=True)
  g.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"))
  g.add_argument("--bookmarks", action="store_true")
  p.add_argument("--before", type=float, default=25.0)
  p.add_argument("--after", type=float, default=10.0)
  p.add_argument("--out", type=Path, default=Path("."))
  args = p.parse_args()

  route = args.route_dir.resolve().name
  args.out.mkdir(parents=True, exist_ok=True)
  if args.window:
    t0, t1 = args.window
    d = extract(args.route_dir, t0 - 1, t1 + 1)
    windows = [(t0, t1, "")]
  else:
    d = extract(args.route_dir, None, None)
    windows = [(b - args.before, b + args.after, f" bookmark {b:.1f} s") for b in d["bookmarks"]]
    if not windows:
      print("no bookmarks in this route")
  mode = "alpha long (openpilot brakes)" if d["params"].get("AlphaLongitudinalEnabled") == "1" else "stock ACC"
  if mode == "stock ACC" and d["params"].get("RedneckCruise") == "1":
    mode += " + ICBM"
  ptxt = mode + " | " + (", ".join(f"{k}={v}" for k, v in sorted(d["params"].items())) or "params not logged")
  for t0, t1, tag in windows:
    path = args.out / f"long_{route}_{int(t0)}_{int(t1)}.png"
    plot(d, t0, t1, f"{route}  {t0:.1f}-{t1:.1f} s{tag}\n{ptxt}", path)
    print(path)


if __name__ == "__main__":
  os.environ.setdefault("MPLBACKEND", "Agg")
  main()
