#!/usr/bin/env python3
"""Open-loop A/B replay of RANGE_VREL_RAIL_NC_CAP (radard) on real rlogs. Scratch harness, not in the repo.

logged CAN -> current Bosch-A RadarInterface -> RadarD x3 (OFF, OFF2 [A/A], ON) -> LongitudinalPlanner x3 (open loop).
Params: a fake Params class backed by the episode segment's own initData params (keys read are recorded).
Time: t = (logMonoTime - seg-0 initData logMonoTime) / 1e9.
"""
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("DEBUG", "0")

PARAM_STORE: dict = {}

from openpilot.selfdrive.controls import radard as RDM
from openpilot.selfdrive.controls.lib import longitudinal_planner as LP
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.longitudinal.alpha_open_loop_replay import (
  _ControlsStateView, _ReplaySM, alpha_car_params, default_toggles, parse_toggles)
from openpilot.tools.longitudinal.alpha_closed_loop_replay import LOG_CLOCK, _RadardSM
from openpilot.common.swaglog import cloudlog
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import radar_interface as HRI

for _l in ("info", "warning", "error", "exception", "event", "debug"):
  if hasattr(cloudlog, _l):
    setattr(cloudlog, _l, lambda *a, **k: None)

SP = Path(os.environ["SP"])  # REQUIRED: work dir outside the repo holding routes/ and outputs (AGENTS.md section 7)
ROUTES = SP.parent / "routes"
PLANNER_SERVICES = ("carState", "starpilotPlan", "selfdriveState", "carControl", "liveParameters",
                    "starpilotCarState", "controlsState")
BOSCH_A_IDS = frozenset(HRI.BOSCH_A_ALL_IDS)
VARIANTS = ("OFF", "OFF2", "ON")
RAIL = RDM.BOSCH_A_U11_LOW_RAIL_MPS + RDM.BOSCH_A_DIRECT_VREL_SCALE_MPS / 2

# NC raw capture: every _bosch_a_nc_vrel call in the current sweep, keyed by the exact dRel float passed.
NC_RAW: dict = {}
_orig_nc = HRI._bosch_a_nc_vrel


def _nc_spy(nc_raw, nc_sigma_raw, d_rel):
  NC_RAW[float(d_rel)] = (None if nc_raw is None else int(nc_raw), None if nc_sigma_raw is None else int(nc_sigma_raw))
  return _orig_nc(nc_raw, nc_sigma_raw, d_rel)


HRI._bosch_a_nc_vrel = _nc_spy


def seg0_init_mono(route):
  for m in LogReader(str(ROUTES / route / "0" / "rlog.zst"), sort_by_time=True):
    if m.which() == "initData":
      return m.logMonoTime, m.initData.gitCommit[:9]
  raise SystemExit("no initData in seg 0")


def lead_d(ld):
  if not ld.status:
    return None
  return {"radar": bool(ld.radar), "tid": int(ld.radarTrackId) if ld.radar else -1, "d": float(ld.dRel),
          "y": float(ld.yRel), "vRel": float(ld.vRel), "vLead": float(ld.vLead), "aLeadK": float(ld.aLeadK)}


def track_d(rd, tid, pts):
  tr = rd.tracks.get(tid)
  if tr is None:
    return None
  out = {"vRel": float(tr.vRel), "ncVRel": float(tr.ncVRel), "ncValid": bool(tr.ncValid), "d": float(tr.dRel),
         "corr": float(tr.range_assist_correction), "on_rail": bool(tr.vRel <= RAIL), "measured": bool(tr.measured),
         "active": bool(tr.range_assist_active)}
  raw = pts.get(tid)
  if raw is not None:
    out["nc_raw"], out["nc_sigma"] = raw
    if raw[0] is not None and raw[0] != HRI.BOSCH_A_NC_CENTER_RAW:
      out["nc_diag_vrel"] = -(raw[0] - HRI.BOSCH_A_NC_CENTER_RAW) * HRI.BOSCH_A_NC_SCALE * float(tr.dRel)
  return out


def run_route(route, segs, windows):
  t0, git0 = seg0_init_mono(route)
  meta = {"route": route, "segs": segs, "t0_seg0": t0, "seg0_git": git0, "seg_counts": {}}
  state, valid = {}, {}
  toggles = default_toggles()
  ri = None
  rds, planners = {}, {}
  prev_alead = dict.fromkeys(VARIANTS, 0.0)
  rsm = _RadardSM()
  rr_latest = None
  last_pts_raw = {}
  frames = []
  LP.time = LOG_CLOCK
  # carParams is logged late in a segment; build the parser/radard/planners from the first one found up front.
  pre_cp = pre_init = None
  for m in LogReader(str(ROUTES / route / str(segs[0]) / "rlog.zst"), sort_by_time=True):
    if m.which() == "initData" and pre_init is None:
      pre_init = m
    if m.which() == "carParams":
      pre_cp = m
      break
  assert pre_init is not None and pre_init.logMonoTime < pre_cp.logMonoTime
  meta["carParams_prescan_t"] = (pre_cp.logMonoTime - t0) / 1e9
  for seg in segs:
    cnt = {"can": 0, "liveTracks": 0, "modelV2": 0, "carState": 0, "bosch_sweeps": 0}
    tmin = tmax = None
    msgs = LogReader(str(ROUTES / route / str(seg) / "rlog.zst"), sort_by_time=True)
    if ri is None:
      import itertools
      msgs = itertools.chain([pre_init, pre_cp], msgs)
    for msg in msgs:
      LOG_CLOCK.now = msg.logMonoTime / 1e9
      w = msg.which()
      if w in cnt:
        cnt[w] += 1
      if w == "can":
        tt = (msg.logMonoTime - t0) / 1e9
        tmin = tt if tmin is None else min(tmin, tt)
        tmax = tt if tmax is None else max(tmax, tt)
      if w == "initData":
        if "init_seg" not in meta:
          params = {e.key: bytes(e.value) for e in msg.initData.params.entries}
          PARAM_STORE.clear()
          PARAM_STORE.update(params)
          meta["init_seg"] = seg
          meta["init_mono_seg"] = msg.logMonoTime
          meta["git"] = msg.initData.gitCommit[:9]
          meta["params_initData"] = {k: (params[k].decode() if k in params else None)
                                     for k in ("BoschARadar", "BlotV3", "NAPAdaptiveAccel")}
        continue
      if w == "carParams" and ri is None:
        cp = msg.carParams
        from opendbc.car.honda.interface import CarInterface
        from opendbc.car.honda.values import CAR
        ocp = CarInterface.get_non_essential_params(getattr(CAR, str(cp.carFingerprint)))
        ocp.radarUnavailable = bool(cp.radarUnavailable)
        ocp.openpilotLongitudinalControl = bool(cp.openpilotLongitudinalControl)
        ri = CarInterface.RadarInterface(ocp)
        meta.update(fingerprint=str(cp.carFingerprint), logged_radarUnavailable=bool(cp.radarUnavailable),
                    logged_op_long=bool(cp.openpilotLongitudinalControl), bosch_a=bool(ri.bosch_a_radar),
                    radarDelay=float(cp.radarDelay), is_bosch_a_radar_car=RDM.is_bosch_a_radar_car(cp))
        assert ri.bosch_a_radar, "parser is not Bosch-A"
        acp = alpha_car_params(cp)
        for v in VARIANTS:
          rds[v] = RDM.RadarD(radar_ts=RDM.DT_MDL, delay=float(cp.radarDelay),
                              honda_bosch_a_radar=RDM.is_bosch_a_radar_car(cp))
          planners[v] = LP.LongitudinalPlanner(acp)
          blot = PARAM_STORE.get("BlotV3", b"0") in (b"1", b"True", b"true")
          planners[v]._blotv3_active = (lambda b=blot: b)  # BLoTv3 from initData, as alpha_open_loop_replay
          meta["planner_is_preap"] = bool(getattr(planners[v], "is_preap", False))
        continue
      if w == "can":
        if ri is not None:
          bosch = [CanData(c.address, bytes(c.dat), c.src) for c in msg.can if c.address in BOSCH_A_IDS]
          if bosch:
            if "carState" in state:
              ri.v_ego = float(state["carState"].vEgo)
            NC_RAW.clear()
            rr = ri.update([(msg.logMonoTime, bosch)])
            if rr is not None:
              cnt["bosch_sweeps"] += 1
              last_pts_raw = {}
              for p in rr.points:
                last_pts_raw[int(p.trackId)] = NC_RAW.get(float(p.dRel))
              rr_latest = rr
              rsm.recv_frame["liveTracks"] += 1
              rsm.logMonoTime["liveTracks"] = msg.logMonoTime
              rsm.valid["liveTracks"] = not any(rr.errors.to_dict().values())
        continue
      if w == "radarState":
        state["radarState_logged"] = msg.radarState
        continue
      if w in PLANNER_SERVICES:
        state[w] = getattr(msg, w)
        valid[w] = bool(msg.valid)
        if w in ("carState", "starpilotPlan"):
          rsm.msgs[w] = state[w]
          rsm.valid[w] = bool(msg.valid)
          rsm.logMonoTime[w] = msg.logMonoTime
          if w == "carState":
            rsm.recv_frame["carState"] += 1
        if w == "starpilotPlan":
          toggles = parse_toggles(state["starpilotPlan"].starpilotToggles, toggles)
        continue
      if w != "modelV2" or not rds or rr_latest is None or not all(k in state for k in PLANNER_SERVICES):
        continue
      model = msg.modelV2
      state["modelV2"] = model
      valid["modelV2"] = bool(msg.valid)
      rsm.msgs["modelV2"] = model
      rsm.valid["modelV2"] = bool(msg.valid)
      rsm.seen["modelV2"] = True
      rsm.logMonoTime["modelV2"] = msg.logMonoTime
      if not meta.get("toggles_init"):
        for v in VARIANTS:  # RadarD.__init__ read host defaults; start from the logged broadcast instead
          rds[v].starpilot_toggles = RDM.get_starpilot_toggles(rsm)
        meta["toggles_init"] = True
      fresh = None
      rs = {}
      for v in VARIANTS:
        RDM.RANGE_VREL_RAIL_NC_CAP = (v == "ON")
        RDM.RANGE_VREL_RAIL_NC_CAP_MARGIN_MPS = float(os.environ.get("NC_MARGIN_ON", "3.0")) if v == "ON" else 3.0
        RDM.get_RadarState_from_vision.prev_aLeadK = prev_alead[v]
        rd = rds[v]
        f_before = rd._last_tracks_frame
        rd.update(rsm, rr_latest)
        prev_alead[v] = getattr(RDM.get_RadarState_from_vision, "prev_aLeadK", 0.0)
        fresh = rsm.recv_frame["liveTracks"] != f_before
        rs[v] = rd.radar_state.as_reader()
      RDM.RANGE_VREL_RAIL_NC_CAP = False
      cs = state["carState"]
      if meta["logged_op_long"]:
        cstate = state["controlsState"]
      else:
        engaged = bool(cs.cruiseState.enabled) and not bool(cs.brakePressed)
        cstate = _ControlsStateView(state["controlsState"], LongCtrlState.pid if engaged else LongCtrlState.off)
      t = (msg.logMonoTime - t0) / 1e9
      fr = {"t": t, "fresh": fresh, "t_lt": (rsm.logMonoTime["liveTracks"] - t0) / 1e9, "aEgo": float(cs.aEgo),
            "vEgo": float(cs.vEgo), "v": {}}
      for v in VARIANTS:
        sm = _ReplaySM(state, valid)
        sm["controlsState"] = cstate
        sm["radarState"] = rs[v]
        sm._valid = {**valid, "radarState": bool(rds[v].radar_state_valid)}
        planners[v].update(sm, toggles)
        l1 = lead_d(rs[v].leadOne)
        rec = {"lead": l1, "a": float(planners[v].output_a_target), "src": str(planners[v].mpc.source)}
        if l1 is not None and l1["radar"]:
          rec["trk"] = track_d(rds[v], l1["tid"], last_pts_raw)
        fr["v"][v] = rec
      lg = state.get("radarState_logged")
      if lg is not None:
        fr["logged_lead"] = lead_d(lg.leadOne)
      if any(a - 4 <= t <= b + 6 for a, b in windows):
        frames.append(fr)
    meta["seg_counts"][seg] = {**cnt, "can_t_span": [tmin, tmax],
                               "can_hz": cnt["can"] / (tmax - tmin) if tmin is not None and tmax > tmin else None}
  return frames, meta


def main():
  route, segs, windows = sys.argv[1], [int(s) for s in sys.argv[2].split(",")], json.loads(sys.argv[3])
  frames, meta = run_route(route, segs, windows)
  out = SP / (f"frames_{route}.json" if "NC_MARGIN_ON" not in os.environ else f"posctl_{route}.json")
  out.write_text(json.dumps({"meta": meta, "frames": frames}))
  print(json.dumps(meta, indent=1, default=str))


if __name__ == "__main__":
  main()
