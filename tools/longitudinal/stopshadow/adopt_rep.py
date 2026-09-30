"""(adapted from leadlate/rep.py) Slim radard replay (no planners) for 00000297 seg 31: per model step, leadOne source and tid 6 / tid 3 match terms."""
import os, sys, pickle
from pathlib import Path
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.controls import radard as RDM
from openpilot.tools.longitudinal import alpha_closed_loop_replay as T
from openpilot.tools.longitudinal.alpha_open_loop_replay import segment_files
from opendbc.car.can_definitions import CanData
route = Path(sys.argv[1]).expanduser(); SEGS = {int(s) for s in sys.argv[2].split(",")}
T0, T1, OUT = float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
TIDS = (int(sys.argv[6]),)
files = segment_files(route)
t0 = None
for m in LogReader(str(files[0])):
  if m.which() == "initData":
    t0 = m.logMonoTime; break
files = [f for f in files if int(f.parent.name) in SEGS]
rsm = T._RadardSM(); ri = rd = rr_latest = None; rows = []; state = {}
def terms(tr, lead, v):
  off = lead.x[0] - RDM.RADAR_TO_CAMERA
  return dict(id=tr.identifier, d=tr.dRel, y=tr.yRel, vrel=tr.vRel, cnt=tr.cnt, meas=tr.measured,
    dd=tr.dRel - off, dlim=max(abs(off) * 0.25, 5.0), dy=tr.yRel + lead.y[0], ylim=max(1.0, max(float(lead.yStd[0]), 0.2)),
    dv=tr.vRel + v - lead.v[0], vlead=v + tr.vRel,
    strict=RDM.track_matches_vision(tr, lead, v, dist_scale=0.25, dist_floor=5.0, vel_limit=10.0, y_std_scale=1.0, y_floor=1.0),
    vtp=RDM.vision_track_probability(tr, lead, v), onp_n=len(tr.onpath_hist), onp=tr.onpath_adoptable(), nou11=adopt_nou11(tr))

def adopt_nou11(tr):
  """onpath_adoptable with the U11 terms removed: closing and rate taken from the range LSQ only (analysis variant)."""
  h = tr.onpath_hist
  if not h or not (tr.dRel <= RDM.ONPATH_ADOPT_MAX_D_REL_M): return (False, 'd')
  tl = h[-1][0]; win = [x for x in h if x[0] >= tl - RDM.ONPATH_ADOPT_MIN_SPAN_S - 1e-6]
  if len(win) < RDM.ONPATH_ADOPT_MIN_SAMPLES or h[0][0] > tl - RDM.ONPATH_ADOPT_MIN_SPAN_S + 1e-6: return (False, 'span')
  offs = np.abs(np.array([x[3] for x in win]))
  if np.median(offs) > RDM.ONPATH_ADOPT_MEDIAN_WIDTH_M or np.mean(offs <= RDM.ONPATH_ADOPT_CORE_WIDTH_M) < RDM.ONPATH_ADOPT_CORE_FRAC: return (False, 'width')
  ts = np.array([x[0] for x in win]) - tl; ds = np.array([x[1] for x in win]); vm = float(np.mean([x[2] for x in win]))
  sl, ic = np.polyfit(ts, ds, 1); rms = float(np.sqrt(np.mean((ds - (sl * ts + ic)) ** 2)))
  if rms > RDM.ONPATH_ADOPT_MAX_RANGE_RESIDUAL_M: return (False, 'rms')
  if sl > -RDM.ONPATH_ADOPT_MIN_CLOSING_MPS: return (False, 'rangeclosing')
  u11_ok = vm <= -RDM.ONPATH_ADOPT_MIN_CLOSING_MPS and sl <= vm + RDM.ONPATH_ADOPT_RATE_TOL_MPS and (sl >= vm - RDM.ONPATH_ADOPT_RATE_TOL_MPS or vm <= RDM.ONPATH_ADOPT_RAIL_VREL_MPS)
  return (True, 'u11ok' if u11_ok else 'u11block')
done = False
for path in files:
  for msg in LogReader(str(path), sort_by_time=True):
    T.LOG_CLOCK.now = msg.logMonoTime / 1e9
    w = msg.which()
    if w == "carParams" and rd is None:
      cp = msg.carParams
      ri, _ = T.build_radar_interface(str(cp.carFingerprint))
      rd = RDM.RadarD(radar_ts=RDM.DT_MDL, delay=float(cp.radarDelay), honda_bosch_a_radar=True); continue
    if w == "can":
      if ri is not None:
        b = [CanData(c.address, bytes(c.dat), c.src) for c in msg.can if c.address in T.BOSCH_A_IDS]
        if b:
          rr = ri.update([(msg.logMonoTime, b)])
          if rr is not None:
            rr_latest = rr; rsm.recv_frame["liveTracks"] += 1; rsm.logMonoTime["liveTracks"] = msg.logMonoTime
            rsm.valid["liveTracks"] = not any(rr.errors.to_dict().values())
      continue
    if w == "radarState":
      state["rs"] = msg.radarState; continue
    if w in T.RADARD_SERVICES and w != "modelV2":
      rsm.msgs[w] = getattr(msg, w); rsm.valid[w] = bool(msg.valid); rsm.logMonoTime[w] = msg.logMonoTime
      if w == "carState": rsm.recv_frame["carState"] += 1
      continue
    if w != "modelV2" or rd is None or rr_latest is None or not all(k in rsm.msgs for k in T.RADARD_SERVICES if k != "modelV2"):
      continue
    md = msg.modelV2
    rsm.msgs["modelV2"] = md; rsm.valid["modelV2"] = bool(msg.valid); rsm.seen["modelV2"] = True; rsm.logMonoTime["modelV2"] = msg.logMonoTime
    t = (msg.logMonoTime - t0) / 1e9
    rd.update(rsm, rr_latest)
    if t > T1: done = True; break
    if t < T0: continue
    L = rd.radar_state.leadOne; On = rd.radar_state.leadOnpath; lead = md.leadsV3[0]; v = rd.v_ego
    best = max(rd.tracks.values(), key=lambda c: RDM.vision_track_probability(c, lead, v)) if rd.tracks else None
    lg = state.get("rs")
    rows.append(dict(ontid=int(On.radarTrackId) if On.status else -1, t=t, v=v, fprob=float(rd.lead_prob_filters[0].x), thr=float(getattr(rd.starpilot_toggles, "lead_detection_probability", 0.35)) if hasattr(rd, "starpilot_toggles") else None,
      vis=(float(lead.prob), float(lead.x[0]), float(lead.y[0]), float(lead.v[0]), float(lead.xStd[0]), float(lead.yStd[0]), float(lead.vStd[0])),
      rep=(bool(L.status), bool(L.radar), int(L.radarTrackId), float(L.dRel)), on=(bool(On.status), int(On.radarTrackId), float(On.dRel)),
      log=(bool(lg.leadOne.status), bool(lg.leadOne.radar), int(lg.leadOne.radarTrackId), float(lg.leadOne.dRel)) if lg else None,
      logon=(bool(lg.leadOnpath.status), int(lg.leadOnpath.radarTrackId), float(lg.leadOnpath.dRel)) if lg else None,
      best=terms(best, lead, v) if best else None, trk={i: terms(rd.tracks[i], lead, v) for i in TIDS if i in rd.tracks}))
  if done: break
pickle.dump(rows, open(OUT, "wb")); print("rows", len(rows), "t0-based")
