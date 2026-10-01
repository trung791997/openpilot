#!/usr/bin/env python3
"""Re-run radard from the logged Bosch-A CAN, once per U11-scale variant, and dump leadOne/leadTwo per modelV2 frame.

REPLAY tool only. Variants (module globals swapped around each variant's RadarInterface.update and RadarD.update;
every use of these constants is a call-time global lookup at 50944ee2a, checked by grep):
  base : U11 at 1/64, LOW_RAIL -13.5, ONPATH_ADOPT_RAIL_VREL_MPS as shipped (-12.5)
  s72  : U11 at 1/72, LOW_RAIL -12.0, ONPATH -12.5 kept (never fires at 1/72)
  s72b : U11 at 1/72, LOW_RAIL -12.0, ONPATH = LOW_RAIL + 1.0 (-11.0), as on branch u11-scale72-toggle
Radard keeps track/arm history, so replay from the route start (or at least a full segment before the window):
a seg-2-only replay of 2a6 did NOT reproduce the 2:40 arm state.

Usage (repo root):
  PARAMS_ROOT=<any writable dir> PYTHONPATH=.:opendbc_repo:tools python tools/longitudinal/sim_kit/radard_from_can.py \
      <route_dir> <out.npz> [--segs 0-3] [--t0 LOGMONO_S --t1 LOGMONO_S] [--variants base,s72,s72b]
Output npz: t (logMono s, modelV2 frames), seg, vEgo, fields (names), and per variant V an array V of shape
(N, 2, len(fields)) for [leadOne, leadTwo]. Frames before the route's first starpilotPlan are skipped (radard reads it).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('route'); ap.add_argument('out')
ap.add_argument('--segs', default=None); ap.add_argument('--t0', type=float, default=None); ap.add_argument('--t1', type=float, default=None)
ap.add_argument('--variants', default='base,s72,s72b')
a = ap.parse_args()
sys.argv = [sys.argv[0]]

import tools.longitudinal.alpha_closed_loop_replay as A  # noqa: E402
from tools.lib.logreader import LogReader  # noqa: E402
from opendbc.car.can_definitions import CanData  # noqa: E402
import opendbc.car.honda.radar_interface as RI  # noqa: E402

RDM = A.RDM
LOW64 = RDM.BOSCH_A_U11_LOW_RAIL_MPS
LOW72 = (RI.BOSCH_A_DIRECT_VREL_MIN_RAW - RI.BOSCH_A_DIRECT_VREL_CENTER_RAW) / 72.0
CFG = {'base': (1 / 64, LOW64, RDM.ONPATH_ADOPT_RAIL_VREL_MPS), 's72': (1 / 72, LOW72, -12.5), 's72b': (1 / 72, LOW72, LOW72 + 1.0)}
M = tuple(a.variants.split(','))
FIELDS = ('status', 'radar', 'dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'modelProb', 'radarTrackId')


def setv(m):
  s, low, onp = CFG[m]
  RI.BOSCH_A_DIRECT_VREL_SCALE_MPS = s
  RDM.BOSCH_A_DIRECT_VREL_SCALE_MPS = s
  RDM.BOSCH_A_U11_LOW_RAIL_MPS = low
  RDM.ONPATH_ADOPT_RAIL_VREL_MPS = onp


segs = sorted((p for p in Path(a.route).iterdir() if p.is_dir() and p.name.isdigit() and (p / 'rlog.zst').exists()), key=lambda p: int(p.name))
if a.segs:
  lo, hi = (int(x) for x in a.segs.split('-'))
  segs = [p for p in segs if lo <= int(p.name) <= hi]
ris, rds, rsm, rr_latest = {}, {}, A._RadardSM(), {}
T, SEG, VE, OUT = [], [], [], {m: [] for m in M}
for sp in segs:
  try:
    for msg in LogReader(str(sp / 'rlog.zst'), sort_by_time=True):
      w = msg.which(); t = msg.logMonoTime / 1e9
      if a.t1 is not None and t > a.t1:
        break
      if w == 'carParams' and not ris:
        cp = msg.carParams
        for m in M:
          setv(m); ris[m], _ = A.build_radar_interface(str(cp.carFingerprint))
        rds = {m: RDM.RadarD(radar_ts=RDM.DT_MDL, delay=float(cp.radarDelay), honda_bosch_a_radar=True) for m in M}
        continue
      if w == 'can' and ris:
        b = [CanData(c.address, bytes(c.dat), c.src) for c in msg.can if c.address in A.BOSCH_A_IDS]
        if b:
          got = False
          for m in M:
            setv(m); rr = ris[m].update([(msg.logMonoTime, b)])
            if rr is not None:
              rr_latest[m] = rr; got = True
          if got:
            rsm.recv_frame['liveTracks'] += 1; rsm.logMonoTime['liveTracks'] = msg.logMonoTime
            rsm.valid['liveTracks'] = not any(rr_latest[M[0]].errors.to_dict().values())
        continue
      if w in A.RADARD_SERVICES:
        rsm.msgs[w] = getattr(msg, w); rsm.valid[w] = bool(msg.valid); rsm.logMonoTime[w] = msg.logMonoTime
        if w == 'carState':
          rsm.recv_frame['carState'] += 1
        continue
      if w != 'modelV2' or not ris or len(rr_latest) < len(M) or 'carState' not in rsm.msgs or 'starpilotPlan' not in rsm.msgs:
        continue
      rsm.msgs['modelV2'] = msg.modelV2; rsm.valid['modelV2'] = True; rsm.seen['modelV2'] = True; rsm.logMonoTime['modelV2'] = msg.logMonoTime
      keep = a.t0 is None or t >= a.t0
      for m in M:
        setv(m); rds[m].update(rsm, rr_latest[m])
        if keep:
          rs = rds[m].radar_state
          OUT[m].append([[float(getattr(l, f)) for f in FIELDS] for l in (rs.leadOne, rs.leadTwo)])
      if keep:
        T.append(t); SEG.append(int(sp.name)); VE.append(rsm.msgs['carState'].vEgo)
  except Exception as e:  # keep going: one bad segment must not drop the rest of the route
    print('seg fail', sp, repr(e)[:160])
  if a.t1 is not None and T and T[-1] > a.t1:
    break
setv('base')
np.savez_compressed(a.out, t=np.array(T), seg=np.array(SEG), vEgo=np.array(VE), fields=np.array(FIELDS), **{m: np.array(OUT[m]) for m in M})
print(Path(a.route).name, 'frames', len(T), 'variants', M)
