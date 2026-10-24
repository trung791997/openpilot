"""D-048: corroboration bounds a radar lead's braking authority; it never deletes the point.

A radar track that migrates onto the wrong scatterer keeps its range and velocity mutually consistent while both
describe the wrong object, so no radar-internal check can see it (D-048, 231 t=9.1-11.35). The only independent
witness is the vision lead. Each radar track keeps:
  - dv_ema: a slow EMA of (radar vLead - vision vLead), negative when radar says the lead is slower;
  - dd_ema: a slow EMA of (vision range - radar range), positive when radar says the lead is closer;
  - trust_s: seconds the track has agreed with vision on measured (not coasted) sweeps, capped at TRUST_FULL_S;
    lost only after DISAGREE_GRACE_S of sustained disagreement, so a trusted lead that really brakes keeps authority
    while the vision velocity catches up, and a trusted track that migrates (231) loses it within ~2.5 s.
When radar is the more severe of the two, the published lead is moved toward vision by a weight w:
  w = W_MAX * f(dv_ema) * (1 - trust) * f(model prob) * f(urgency)
Never toward the more severe side, never all the way (W_MAX < 1, D-042), never a step (every term is an EMA or a
ramp), and the point is always published (D-041). Urgent geometry on the radar's own numbers ramps w to zero.

Evidence for the numbers (limited road evidence, two faults, NOT yet the D-048 validation gate):
  231 t=9.1-11.35: radar 71.8->61.6 m, U11 to -6.0 m/s; vision 75-78 m at prob >= 0.93; plan -2.89, driver overrode.
  20 (11c8fa231c0499ed/00000020--b3f743be5f) T285-287: fresh id23 at 73.7 m read 54-64 m, vLead ~16 m/s; vision
    58-67 m at 21-25 m/s; qcamera puts the in-lane truck at ~72 m; aLeadK -3, plan -3.5, aEgo -4.7, driver overrode.
  Both are dv <= -4 m/s for > 1 s at d >= 50 m with TTC > 5 s on the radar's own numbers.
Range alone was refuted as a trigger (p50 |gap| 4.8-14.9 m at 60-80 m), so the trigger is dv; dd only rides along,
smoothed, once dv has already opened the bound. The constants below are provisional until the corpus distribution
of dv by range and track age (D-048 gate item 1) is in.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openpilot.common.realtime import DT_MDL

RADAR_TO_CAMERA = 1.52

DV_TAU_S = 0.8                # dv/dd EMA time constant: slower than vision v/x jitter, faster than the 2-3 s faults
AGREE_DV = 2.0                # |dv| below this on a measured sweep counts as agreement
TRUST_FULL_S = 3.0            # seconds of agreement for full trust (no bound at all)
DISAGREE_GRACE_S = 1.0        # a trusted track keeps full trust this long into a disagreement (vision v lags real braking)
TRUST_DECAY = 2.0             # then trust is lost at this many seconds per second (full trust gone in 1.5 s)
DV_BP = [-2.5, -5.0]          # dv_ema where the bound starts / reaches W_MAX
W_MAX = 0.6                   # never more than 60 % of the way to vision (D-042: blended, never switched)
PROB_BP = [0.7, 0.9]          # filtered model lead prob where vision starts / fully counts as a witness
URGENT_D_BP = [35.0, 50.0]    # radar range (m): below 35 no bound, above 50 full bound
URGENT_TTC_BP = [4.0, 6.0]    # radar TTC (s): below 4 no bound, above 6 full bound


@dataclass
class TrackCorroboration:
  dv_ema: float = 0.0
  dd_ema: float = 0.0
  trust_s: float = 0.0
  disagree_s: float = 0.0
  primed: bool = False


def urgency_factor(d_rel: float, v_rel: float) -> float:
  """1.0 when the radar's own geometry is not urgent, ramping to 0.0 as range or TTC gets short."""
  closing = max(-v_rel, 0.0)
  ttc = d_rel / closing if closing > 0.1 else math.inf
  f_d = float(np.interp(d_rel, URGENT_D_BP, [0.0, 1.0]))
  f_ttc = 1.0 if math.isinf(ttc) else float(np.interp(ttc, URGENT_TTC_BP, [0.0, 1.0]))
  return min(f_d, f_ttc)


class RadarCorroboration:
  """Per-track corroboration state and the bounded lead. One instance per BoschALeadPolicy."""
  def __init__(self, dt: float = DT_MDL):
    self.alpha = dt / (DV_TAU_S + dt)
    self.dt = dt
    self.state: dict[int, TrackCorroboration] = {}
    self.weights = [0.0, 0.0]   # last applied w per lead index, for replay/debug

  def prune(self, track_ids) -> None:
    for identifier in [i for i in self.state if i not in track_ids]:
      del self.state[identifier]

  def observe(self, track, lead_msg, v_ego: float, model_v_ego: float, lead_prob: float) -> TrackCorroboration:
    """Update the track's corroboration against the vision lead it was matched to. Call once per frame per track."""
    s = self.state.setdefault(track.identifier, TrackCorroboration())
    vision_v = v_ego + (float(lead_msg.v[0]) - model_v_ego)
    vision_d = float(lead_msg.x[0]) - RADAR_TO_CAMERA
    dv = float(track.vLead) - vision_v
    dd = vision_d - float(track.dRel)
    if not s.primed:
      s.dv_ema, s.dd_ema, s.primed = dv, dd, True
    else:
      s.dv_ema += self.alpha * (dv - s.dv_ema)
      s.dd_ema += self.alpha * (dd - s.dd_ema)
    if abs(s.dv_ema) >= AGREE_DV:
      s.disagree_s += self.dt
      if s.disagree_s > DISAGREE_GRACE_S:
        s.trust_s = max(s.trust_s - TRUST_DECAY * self.dt, 0.0)
    else:
      s.disagree_s = 0.0
      if bool(getattr(track, 'measured', True)) and abs(dv) < AGREE_DV and lead_prob >= PROB_BP[1]:
        s.trust_s = min(s.trust_s + self.dt, TRUST_FULL_S)
    return s

  def weight(self, s: TrackCorroboration, lead: dict, lead_prob: float) -> float:
    f_dv = float(np.interp(-s.dv_ema, [-DV_BP[0], -DV_BP[1]], [0.0, 1.0]))
    f_trust = 1.0 - s.trust_s / TRUST_FULL_S
    f_prob = float(np.interp(lead_prob, PROB_BP, [0.0, 1.0]))
    return W_MAX * f_dv * f_trust * f_prob * urgency_factor(float(lead['dRel']), float(lead['vRel']))

  def bound(self, index: int, lead: dict, lead_msg, v_ego: float, model_v_ego: float,
            lead_prob: float) -> dict:
    """The published lead: `lead` itself unless it is a radar lead that vision says is less severe."""
    self.weights[index] = 0.0
    if not (lead.get('present', False) and lead.get('radar', False)):
      return lead
    s = self.state.get(int(lead.get('radarTrackId', -1)))
    if s is None or not s.primed:
      return lead
    w = self.weight(s, lead, lead_prob)
    if w <= 0.0:
      return lead
    out = dict(lead)
    vision_v = v_ego + (float(lead_msg.v[0]) - model_v_ego)
    dv_shift = w * max(vision_v - float(lead['vLead']), 0.0)
    out['vLead'] = float(lead['vLead']) + dv_shift
    out['vLeadK'] = float(lead['vLeadK']) + dv_shift
    out['vRel'] = float(lead['vRel']) + dv_shift
    out['aLeadK'] = float(lead['aLeadK']) + w * max(float(lead_msg.a[0]) - float(lead['aLeadK']), 0.0)
    # Range only ever moves out, and only by the smoothed gap: the raw vision x is too jittery at range (D-048).
    out['dRel'] = float(lead['dRel']) + w * max(s.dd_ema, 0.0)
    self.weights[index] = w
    return out
