"""starpilotRadarState.leadLeft / leadRight producer, ported from openpilot-radar `iq-nav-on-pr10`
selfdrive/controls/radard.py (`get_adjacent_lead`, `Track.potential_adjacent_lead`, the ADJACENT_RAIL_GATE /
ADJACENT_RAIL_CONFIRM latch and the short range fit it reads).

Scope on this tree: the only consumer of leadLeft/leadRight is the V-ASM radar pass hold
(starpilot/spot_monitor/radar_pass_hold.py, toggle VASMRadarAssist). Nothing in longitudinal or lateral control reads
them; qualifiedAdjacent (conditional mode) and leadOne/leadTwo are produced elsewhere and are not touched here. This is
a labelling layer only: it never removes a track from self.tracks or from lead selection (D-041/D-042).

Differences from the source, all forced by 7.0's radard:
  * 7.0's Track has no range history, so the 5-sample range fit (vRelRange / vRelRangeFresh) and the rail latch live
    here, per track id, fed from the same measured updates the source fed them from (Track.measured on Bosch-A,
    t_now = radarTracks logMonoTime as the source's liveTracks logMonoTime).
  * The source excludes `Track.leadTrackID`, which its get_lead overwrites on every track at each call, so after the
    leadOne and leadTwo calls it holds leadTwo's radarTrackId (-1 for a vision-only or absent leadTwo). The caller
    passes that same id here, so the exclusion is the source's, quirk included.
  * D-089's RECOVERED_CAM_GATE has nothing to filter: 7.0's bosch_a_radar.py ships BOSCH_A_REANCHOR_RECOVER_SIGMA_FRAC
    = 0 and RadarPoint has no `recovered` flag.
  * The published dict carries the native vRel/vLead/vLeadK. The source applies D-053's range_assist_correction (zero
    except on the armed lead track) and D-077's birth-rail ramp (off by default); 7.0 has neither.

Static/unit-tested only. Not replayed against the source on a real route and not road-validated on this tree.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Any

import numpy as np

from opendbc.car.honda.bosch_a_radar import (BOSCH_A_DIRECT_VREL_CENTER_RAW, BOSCH_A_DIRECT_VREL_MIN_RAW,
                                             BOSCH_A_DIRECT_VREL_SCALE_MPS)


# Shadow range-derived vRel, computed for whichever radar lead is selected -- not only Bosch-A.
# Timestamps come from the message clock so replay is faithful; wall-clock time made every
# accelerated replay of this field meaningless. The fit is plain LSQ with no outlier rejection, so
# it inherits the range channel's ~1% gross outliers: read it as a diagnostic, not as a validated
# velocity.
#
# Telemetry only (D-044) EXCEPT on Bosch-A with RANGE_VREL_ASSIST on (built in), where D-053 lets it
# correct the published lead velocity in one direction. The outlier caveat above is precisely why
# that assist needs RANGE_VREL_ASSIST_ARM_UPDATES; see the block below.
# (Port note: on this tree the fit feeds only the adjacent-lead rail gate below; there is no D-053 here.)
RANGE_VREL_SAMPLES = 5   # deque length AND the minimum fit length; do not diverge these
RANGE_VREL_MIN_SPAN_S = 0.12
RANGE_VREL_MAX_SPAN_S = 0.60

# Minimum disagreement, m/s, before arming. D-043 deliberately does not chase "the milder
# 0.6-2.5 m/s overshoots at deceleration onset"; this is the mirror of that, and 2.0 m/s is also
# the band tools/bosch_a_vrel_shadow_report.py already reports against. It is NOT a noise floor on
# its own: with 5 evenly spaced samples the LSQ slope moves 0.2/h per metre of error on the newest
# sample, h = 1/14.35 s, so a single 1 m range outlier shifts the fit ~2.9 m/s and clears this on
# its own. ARM_UPDATES is what rejects that, not this threshold.
RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS = 2.0

# D-074: the U11 scale radard assumes must match the one radar_interface decodes with, 1/72 (-12.0 m/s rail). The
# -13.50 / -13.5 rail values quoted in this file come from routes logged under the old 1/64 decode (x 64/72 for 1/72
# units); thresholds were not re-tuned.
BOSCH_A_U11_SCALE_MPS = BOSCH_A_DIRECT_VREL_SCALE_MPS
BOSCH_A_U11_LOW_RAIL_MPS = (BOSCH_A_DIRECT_VREL_MIN_RAW - BOSCH_A_DIRECT_VREL_CENTER_RAW) * BOSCH_A_U11_SCALE_MPS

# --- Adjacent-lead rail gate (2026-09-28). REPLAY evidence only, nothing driven.
# The range assist above only runs on leadOne/leadTwo, so every other track publishes the raw U11
# rail: vLead = vEgo - 12.0 even for a stopped object (-13.5 in the 1/64 logs below). 0000028f seg 6 ~30.5-34 s (UI Work, STATUS
# 108): track 49 at yRel ~3.2, range 74.6 -> 7.9 m in 3.5 s (closing ~ vEgo 21 m/s, i.e. stationary),
# U11 pinned at -13.5 (1/64) throughout, was published as leadLeft at ~16 mph; track 43 (yRel ~9) the same.
# leadLeft/leadRight feed the UI and the conditional-chill adjacent-lead veto.
# A Bosch-A track stops being ELIGIBLE as leadLeft/leadRight once, on the rail, the fresh short range
# fit says at least RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS more closing than the rail on
# ADJACENT_RAIL_GATE_UPDATES consecutive measured updates; it stays so until U11 leaves the rail.
# The point itself is still published and still eligible as leadOne/leadTwo (D-041/D-042); only
# the adjacent-lead label changes. Set ADJACENT_RAIL_GATE to False to restore the old behaviour.
# (Port note: on this tree leadLeft/leadRight feed only the V-ASM radar pass hold; see the module docstring.)
ADJACENT_RAIL_GATE = True
ADJACENT_RAIL_GATE_UPDATES = 3
# ADJACENT_RAIL_CONFIRM (2026-10-03, owner-approved, ships ON). REPLAY evidence only (Job, 000002cb segs 8-12):
# the latch above needs ADJACENT_RAIL_GATE_UPDATES fresh fits, but 109 railed leadLeft holds there lasted a median
# 0.3 s (max 3.0 s), long enough to draw an oncoming car as a speed-labelled side lead. Their vLead = vEgo + rail was
# positive (median 10.7), so the vLead < 1 check passed them; 78 of the 82 with a fit closed at -15 or faster.
# So a Bosch-A track on the rail is eligible as leadLeft/leadRight only while a fresh short range fit agrees with the
# rail (vRel - vRelRange < RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS). No fit yet -> not eligible; cost: a real railed
# side car stays unlabelled for a few sweeps (27 of the 109 had no fit). As with the gate, the point is still
# published and still eligible as leadOne/leadTwo (D-041/D-042). False restores the latch-only behaviour.
ADJACENT_RAIL_CONFIRM = True


class AdjacentRailState:
  """The source Track's range_hist / vRelRange / vRelRangeFresh / rail_range_* fields, for one track."""

  def __init__(self):
    self.range_hist: deque = deque(maxlen=RANGE_VREL_SAMPLES)
    self.vRelRange = float('nan')
    # Freshness bit for the fit above: vRelRange is only ASSIGNED once the deque is full, so after a dropout clears
    # it the field holds the pre-gap value for up to four updates.
    self.vRelRangeFresh = False
    self.rail_range_count = 0
    self.rail_range_inconsistent = False

  def update(self, d_rel: float, v_rel: float, measurement_update: bool, t_now: float) -> None:
    if not measurement_update:
      return  # a duplicate payload would forge a zero-dt sample, and must not advance the latch
    self.vRelRangeFresh = False
    self.range_hist.append((float(t_now), float(d_rel)))
    if len(self.range_hist) >= RANGE_VREL_SAMPLES:
      ts = np.array([p[0] for p in self.range_hist])
      ds = np.array([p[1] for p in self.range_hist])
      span = ts[-1] - ts[0]
      # Reject a stale or gappy history: an LSQ across a dropout is meaningless.
      if RANGE_VREL_MIN_SPAN_S <= span <= RANGE_VREL_MAX_SPAN_S:
        self.vRelRange = float(np.polyfit(ts - ts[-1], ds, 1)[0])
        self.vRelRangeFresh = True
      else:
        self.vRelRange = float('nan')
        if span > RANGE_VREL_MAX_SPAN_S:
          self.range_hist.clear()
          self.range_hist.append((float(t_now), float(d_rel)))
    self._update_rail_range_inconsistent(float(v_rel))

  def _update_rail_range_inconsistent(self, v_rel: float) -> None:
    """ADJACENT_RAIL_GATE latch. Rail-agnostic here; only Bosch-A callers act on it (a rail-valued vRel is a
    real reading on other radars)."""
    if v_rel > BOSCH_A_U11_LOW_RAIL_MPS + BOSCH_A_U11_SCALE_MPS / 2:
      self.rail_range_count = 0
      self.rail_range_inconsistent = False
      return
    if self.vRelRangeFresh and v_rel - self.vRelRange >= RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS:
      self.rail_range_count += 1
      if self.rail_range_count >= ADJACENT_RAIL_GATE_UPDATES:
        self.rail_range_inconsistent = True
    else:
      self.rail_range_count = 0

  def rail_unconfirmed(self, v_rel: float) -> bool:
    """ADJACENT_RAIL_CONFIRM: U11 on the rail and no fresh range fit agreeing with it. Bosch-A callers only."""
    if v_rel > BOSCH_A_U11_LOW_RAIL_MPS + BOSCH_A_U11_SCALE_MPS / 2:
      return False
    agrees = (self.vRelRangeFresh and math.isfinite(self.vRelRange) and
              v_rel - self.vRelRange < RANGE_VREL_ASSIST_MIN_DISAGREEMENT_MPS)
    return not agrees


class AdjacentRailMonitor:
  """Per-track AdjacentRailState, keyed by track id and tied to the Track object, so a reused id or a track radard
  recreated starts a fresh history (the source kept this state on the Track itself)."""

  def __init__(self):
    self.states: dict[int, tuple[Any, AdjacentRailState]] = {}

  def update(self, tracks: dict[int, Any], t_now: float) -> None:
    """Call once per radard cycle, after the tracks were updated. measurement_update is the track's `measured` bit
    (7.0's Honda Track stores measured == measurement_update); tracks without it count every cycle as measured."""
    for ident in list(self.states):
      if ident not in tracks or self.states[ident][0] is not tracks[ident]:
        del self.states[ident]
    for ident, track in tracks.items():
      entry = self.states.get(ident)
      if entry is None:
        entry = (track, AdjacentRailState())
        self.states[ident] = entry
      entry[1].update(track.dRel, track.vRel, bool(getattr(track, 'measured', True)), t_now)

  def state(self, track: Any) -> AdjacentRailState | None:
    entry = self.states.get(track.identifier)
    return entry[1] if entry is not None and entry[0] is track else None


def potential_adjacent_lead(track: Any, left: bool, standstill: bool, model_data: Any, lead_track_id: int) -> bool:
  if standstill or track.vLead < 1 or lead_track_id == track.identifier:
    return False

  if left:
    left_lane = np.interp(track.dRel, model_data.laneLines[1].x, model_data.laneLines[1].y)
    return -track.yRel < left_lane
  right_lane = np.interp(track.dRel, model_data.laneLines[2].x, model_data.laneLines[2].y)
  return -track.yRel > right_lane


def _rail_blocked(track: Any, monitor: AdjacentRailMonitor | None) -> bool:
  state = monitor.state(track) if monitor is not None else None
  if state is None:
    state = AdjacentRailState()  # no history yet: no fit, so the confirm rule treats a railed track as unconfirmed
  return state.rail_range_inconsistent or (ADJACENT_RAIL_CONFIRM and state.rail_unconfirmed(float(track.vRel)))


def adjacent_lead_state(track: Any) -> dict[str, Any]:
  """custom.capnp StarPilotRadarState.LeadData for a track, as the source's Track.get_RadarState() (model_prob 0)."""
  return {
    "dRel": float(track.dRel),
    "yRel": float(track.yRel),
    "vRel": float(track.vRel),
    "vLead": float(track.vLead),
    "vLeadK": float(track.vLeadK),
    "aLeadK": float(track.aLeadK),
    "aLeadTau": float(track.aLeadTau.x),
    "status": True,
    "fcw": False,  # source: is_potential_fcw(model_prob=0.0) -> 0.0 > 0.9
    "modelProb": 0.0,
    "radar": True,
    "radarTrackId": int(track.identifier),
  }


def get_adjacent_lead(tracks: dict[int, Any], standstill: bool, model_data: Any, left: bool = True,
                      honda_bosch_a: bool = False, monitor: AdjacentRailMonitor | None = None,
                      lead_track_id: int = -1) -> dict[str, Any]:
  lead_dict: dict[str, Any] = {'status': False}
  if len(model_data.laneLines) < 3:
    return lead_dict  # port guard: no lane geometry, no side label (the source assumed the model's 4 lane lines)

  rail_gate = ADJACENT_RAIL_GATE and honda_bosch_a
  adjacent_tracks = [c for c in tracks.values()
                     if potential_adjacent_lead(c, left, standstill, model_data, lead_track_id) and
                     not (rail_gate and _rail_blocked(c, monitor))]
  if len(adjacent_tracks) > 0:
    closest_track = min(adjacent_tracks, key=lambda c: c.dRel)
    lead_dict = adjacent_lead_state(closest_track)

  return lead_dict
