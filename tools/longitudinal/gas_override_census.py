#!/usr/bin/env python3
"""Census of driver gas overrides while engaged: was the e2e model the limiting target?

Question it answers: would a gas-override "accel boost" on the e2e target (commaai/openpilot
PR 39015) have anything to fix on our routes? For every rising edge of carState.gasPressed while
selfdriveState.enabled and above MIN_SPEED, it reads the 1 s before the press and reports:

  * experimentalMode, the e2e target (modelV2.action.desiredAcceleration), an MPC estimate
    (get_accel_from_plan on longitudinalPlan.speeds/accels at the actuator delay), aTarget, aEgo
  * the lead (radarState.leadOne): status, radar, dRel, vRel, aLeadK

Class per press (evaluated on the median of the 1 s window):
  e2e_slow_accel   exp mode, e2e < mpc - MARGIN, e2e >= 0          <- what the PR boosts
  e2e_brake        exp mode, e2e < mpc - MARGIN, e2e < 0           <- phantom/over-brake by model
  mpc_brake        aTarget < 0 and not e2e-limited                 <- lead/cruise braking
  mpc_slow         aTarget >= 0 and not e2e-limited
"lead_accel" marks a lead pulling away (status and (aLeadK > 0.3 or vRel > 0.5)).

Usage: gas_override_census.py ROUTE_DIR [ROUTE_DIR ...]   (route dirs contain <seg>/rlog.zst)
Offline log analysis only.
"""
from __future__ import annotations

import argparse
import os
import sys
from collections import Counter

import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.tools.lib.logreader import _LogFileReader

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

MIN_SPEED = 4.5      # m/s, the PR's 10 mph floor
MARGIN = 0.15        # m/s^2, e2e must sit this far under the MPC estimate to count as limiting
WINDOW = 1.0         # s before the press
ACTION_T = 0.5       # s, approximate actuator delay + DT_MDL for the MPC estimate
REARM = 3.0          # s, presses closer than this to the previous one merge into one episode


def segments(route_dir):
  """rlog per segment, else qlog (qlogs carry no modelV2, so e2e reads NaN and nothing classes as e2e-limited)."""
  out = []
  for s in sorted((s for s in os.listdir(route_dir) if s.isdigit()), key=int):
    for name in ("rlog.zst", "qlog.zst"):
      if os.path.exists(os.path.join(route_dir, s, name)):
        out.append(os.path.join(route_dir, s, name))
        break
  return out


def scan_route(route_dir):
  st = dict(enabled=False, exp=False, v=0.0, a=0.0, gas=False, e2e=np.nan, mpc=np.nan, at=np.nan,
            ls=False, lr=False, ld=np.nan, lv=np.nan, la=np.nan)
  hist, events, t0, last_press = [], [], None, -1e9
  for path in segments(route_dir):
    try:
      reader = _LogFileReader(path)
      for m in reader:
        w = m.which()
        t = m.logMonoTime / 1e9
        if t0 is None:
          t0 = t
        if w == "selfdriveState":
          st["enabled"] = m.selfdriveState.enabled
          st["exp"] = m.selfdriveState.experimentalMode
        elif w == "modelV2":
          st["e2e"] = m.modelV2.action.desiredAcceleration
        elif w == "longitudinalPlan":
          lp = m.longitudinalPlan
          st["at"] = lp.aTarget
          if len(lp.speeds) == CONTROL_N:
            st["mpc"] = get_accel_from_plan(list(lp.speeds), list(lp.accels), CONTROL_N_T_IDX, action_t=ACTION_T)[0]
        elif w == "radarState":
          l = m.radarState.leadOne
          st.update(ls=l.status, lr=l.radar, ld=l.dRel, lv=l.vRel, la=l.aLeadK)
        elif w == "carState":
          cs = m.carState
          st["v"], st["a"] = cs.vEgo, cs.aEgo
          rising = cs.gasPressed and not st["gas"]
          st["gas"] = cs.gasPressed
          hist.append((t, dict(st)))
          while hist and hist[0][0] < t - WINDOW:
            hist.pop(0)
          if rising and t - last_press > REARM:
            pre = [h for _, h in hist[:-1]]
            if pre and all(h["enabled"] for h in pre) and st["v"] >= MIN_SPEED:
              events.append((t - t0, os.path.basename(os.path.dirname(path)), pre))
          if rising:
            last_press = t
    except Exception as e:  # truncated segment
      print(f"  warn {path}: {e}", file=sys.stderr)
  return events


def profile_route(route_dir):
  """Engaged-time mix for one route: openpilot long, Experimental Mode share, radar share of lead time."""
  op_long, radar_unavail, enabled, exp, last_t = None, None, False, False, None
  n = Counter()
  eng_s = 0.0
  for path in segments(route_dir):
    try:
      for m in _LogFileReader(path):
        w = m.which()
        if w == "carParams" and op_long is None:
          op_long, radar_unavail = m.carParams.openpilotLongitudinalControl, m.carParams.radarUnavailable
        elif w == "selfdriveState":
          enabled, exp = m.selfdriveState.enabled, m.selfdriveState.experimentalMode
          if not enabled:
            last_t = None
        elif w == "radarState" and enabled:
          t = m.logMonoTime / 1e9
          eng_s += min(t - last_t, 1.0) if last_t is not None else 0.0
          last_t = t
          l = m.radarState.leadOne
          n["eng"] += 1
          n["exp"] += exp
          n["lead"] += l.status
          n["radar"] += l.status and l.radar
          n["exp_lead"] += exp and l.status
          n["exp_radar"] += exp and l.status and l.radar
    except Exception as e:  # truncated segment
      print(f"  warn {path}: {e}", file=sys.stderr)
  pct = lambda a, b: 100.0 * n[a] / n[b] if n[b] else float("nan")  # noqa: E731
  return dict(op_long=op_long, radar_unavail=radar_unavail, eng_min=eng_s / 60, exp_pct=pct("exp", "eng"),
              radar_pct=pct("radar", "lead"), exp_radar_pct=pct("exp_radar", "exp_lead"))


def summarize(pre):
  def med(k):
    return float(np.nanmedian([h[k] for h in pre]))

  exp = sum(h["exp"] for h in pre) > len(pre) / 2
  e2e, mpc, at = med("e2e"), med("mpc"), med("at")
  lead = sum(h["ls"] for h in pre) > len(pre) / 2
  la, lv, ld = med("la"), med("lv"), med("ld")
  e2e_lim = exp and e2e < mpc - MARGIN
  if e2e_lim:
    cls = "e2e_slow_accel" if e2e >= 0 else "e2e_brake"
  else:
    cls = "mpc_brake" if at < 0 else "mpc_slow"
  lead_accel = lead and (la > 0.3 or lv > 0.5)
  return dict(cls=cls, exp=exp, e2e=e2e, mpc=mpc, at=at, a=med("a"), v=med("v"), lead=lead,
              radar=lead and sum(h["lr"] for h in pre) > len(pre) / 2, ld=ld, lv=lv, la=la, lead_accel=lead_accel)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--profile", action="store_true", help="print the per-route engaged-time mix instead of the census")
  args = ap.parse_args()
  if args.profile:
    print(f"{'route':22} opLong radarUnavail eng_min exp%  radar%_of_lead  radar%_of_lead_in_exp")
    for r in args.routes:
      p = profile_route(r)
      print(f"{os.path.basename(os.path.normpath(r)):22} {int(bool(p['op_long'])):6d} {int(bool(p['radar_unavail'])):12d} "
            f"{p['eng_min']:7.1f} {p['exp_pct']:5.1f} {p['radar_pct']:14.1f} {p['exp_radar_pct']:21.1f}")
    return
  total, by_cls, by_cls_lead = Counter(), Counter(), Counter()
  rows = []
  for r in args.routes:
    rid = os.path.basename(os.path.normpath(r))
    for rt, seg, pre in scan_route(r):
      s = summarize(pre)
      rows.append((rid, seg, rt, s))
      by_cls[s["cls"]] += 1
      if s["lead_accel"]:
        by_cls_lead[s["cls"]] += 1
      total["exp" if s["exp"] else "acc"] += 1
  print(f"{'route':22} seg  route_t  cls             exp  v     aEgo   aTgt   e2e    mpc    lead  dRel   vRel   aLeadK")
  for rid, seg, rt, s in rows:
    lead = ('R' if s['radar'] else ('V' if s['lead'] else '-')) + ('+' if s['lead_accel'] else ' ')
    print(f"{rid:22} {seg:>3} {int(rt // 60):4d}:{rt % 60:04.1f} {s['cls']:15} {int(s['exp']):3d} {s['v']:5.1f} {s['a']:6.2f} " +
          f"{s['at']:6.2f} {s['e2e']:6.2f} {s['mpc']:6.2f} {lead:>5} {s['ld']:5.1f} {s['lv']:6.2f} {s['la']:6.2f}")
  print(f"\npresses: {len(rows)}  (exp mode {total['exp']}, acc {total['acc']})")
  for c in ("e2e_slow_accel", "e2e_brake", "mpc_brake", "mpc_slow"):
    print(f"  {c:15} {by_cls[c]:4d}   with lead pulling away: {by_cls_lead[c]}")


if __name__ == "__main__":
  main()
