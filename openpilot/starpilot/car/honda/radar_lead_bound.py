"""Bosch-A: bound the braking authority of a radar lead that the radar or the camera does not back.

Two far false brake taps behind a lead that was not slowing (2026-10-26 drive), each with its own tell:

Route 34 (11c8fa231c0499ed/00000034--010fbd6343) T479, 16 m/s: radar track 55 was born at 119 m and its range walked
119 -> 67 -> 90 -> 52 m while U11 read -10.6 m/s against a RANGE_RATIO of about -1.5. It became lead0 through the
strict gate at 62-76 m with aLeadK already at the -3 clamp; the plan went to -1.91. The model lead held 61-64 m at
13-14 m/s with a ~ 0. For the first 4.2 s of its life the track's F0 STATUS nibble was 9: lead0 from -0.40 s, the
plan's -1.91 low at +1.25 s, STATUS 7 at +1.48 s. Every other track in the seven packs, the leads of the four real far
brakes (2f T736, 10 T188, 32 T1248, 0f T2190) included, read 7 throughout, new ones from their first sweep; the only
other non-7 track (2f track 6, STATUS 3) never became a lead.

Route 37 (00000037--34e868f7ba) T903, 30 m/s: an SUV 45-50 m ahead moving out to the right. Radar track 32 (STATUS 7,
old) dipped vLead 28.7 -> 24.8 (aLeadK -2.9) and went back to 31; the model lead held v 29, a 0 and moved on to the
next car at 61-71 m. Track 32 kept lead0 only through the relaxed gate: from +0.03 s, the start of the tap (plan
-0.29 -> -1.84), its range was more than the strict window (0.25 x range, at least 5 m) short of the model lead's.

That range gap alone is not a tell. On the same drive (T273-278) a pickup slowing to turn left held lead0 at 82 m
while the model lead had moved on to 110 m: the gap reached the window's edge and the plan went to -2.21 for a
real slowdown. There the model lead was itself closing at 11.7 m/s. In the four real far brakes it closed at 4.5 m/s
or more (median 9-13) and the gap stayed under 0.65 of the window; at route 37 T903 it closed at -0.2 to +1.5 m/s. So
the gap counts only while the model lead is past the radar lead and calls for no braking (camera_passed_ratio). On
route 37 none of the six real slowdowns on the stills then gets any weight at its brake; T323, an SUV turning right at
an intersection (ambiguous on the stills), gets at most 0.27 around its -1.12 (log emulation, not a replay).

So while a published radar lead is unconfirmed (STATUS in UNCONFIRMED_STATUSES), or the model lead has moved on past
it as above, its vLead/vLeadK/vRel and aLeadK are moved part of the way toward the model lead by the curve-brake
bound's blend (radar_reversal_bound.ReversalBound.apply, floor_weights): only toward the less severe side, only with
the model lead as a credible witness, never when the radar lead is urgent, dRel never moved, the point always
published (D-041/D-042). Neither tell reads the radar lead's dynamics: the mismatch, cut-out and onset rules
(PROGRESS-7.0 entry 8) did, and also fired on real slowdowns.

The weight is per lead index and rate limited so that a tell coming and going does not step the published lead
(D-042): a new radar track takes its tell's weight at once, an established one rises over RISE_S and releases over
RELEASE_S. Static and pack evidence only; replay pending.
"""
from __future__ import annotations

from typing import Any

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.car.honda.radar_reversal_bound import WITNESS_PROB, urgent

UNCONFIRMED_STATUSES = frozenset({9})  # route 34's phantom; 7 = confirmed, 0 = not provided (other radars, old logs)
UNCONFIRMED_W = 0.6
DISTANCE_RATIO_START = 0.75  # real far brakes stayed under 0.65; route 37 crossed 0.77 -> 1.02 at the tap's start
DISTANCE_RATIO_FULL = 1.0    # the strict gate's edge
DISTANCE_W = 0.6
CAMERA_CLOSING_MAX = 2.0     # m/s; route 37 T903 -0.2 to +1.5, real far brakes 4.5 or more, the turning pickup 11.7
CAMERA_DECEL_MIN = -0.5      # m/s^2; a model lead slowing harder than this calls for braking itself
RISE_S = 0.25
RELEASE_S = 1.0


def camera_passed_ratio(d_rel: float, camera_d_rel: float, camera_closing: float, camera_a: float) -> float:
  """How far the model lead is past the radar lead, as a fraction of radard's strict distance window (0.25 x range, at
  least 5 m; 1.0 = its edge). 0 unless the model lead is past it and calls for no braking: closing at most
  CAMERA_CLOSING_MAX and slowing no harder than CAMERA_DECEL_MIN. camera_d_rel is from the radar, like d_rel."""
  if camera_d_rel <= d_rel or camera_closing > CAMERA_CLOSING_MAX or camera_a < CAMERA_DECEL_MIN:
    return 0.0
  return (camera_d_rel - d_rel) / max(abs(camera_d_rel) * 0.25, 5.0)


def tell_weight(object_status: int, distance_ratio: float) -> float:
  unconfirmed = UNCONFIRMED_W if int(object_status) in UNCONFIRMED_STATUSES else 0.0
  ramp = (float(distance_ratio) - DISTANCE_RATIO_START) / (DISTANCE_RATIO_FULL - DISTANCE_RATIO_START)
  return max(unconfirmed, DISTANCE_W * min(max(ramp, 0.0), 1.0))


class WeakLeadBound:
  """Per lead index weight for the published Bosch-A leads. Call `update` once per radard cycle, before the bound."""
  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.track_ids = [-1, -1]
    self.weights = [0.0, 0.0]

  def update(self, output: list[dict[str, Any]], object_statuses, distance_ratios, lead_probs) -> list[float]:
    rise = max(UNCONFIRMED_W, DISTANCE_W) * self.dt / RISE_S
    release = max(UNCONFIRMED_W, DISTANCE_W) * self.dt / RELEASE_S
    for index, lead in enumerate(output):
      track_id = int(lead.get('radarTrackId', -1)) if lead.get('present', False) and lead.get('radar', False) else -1
      if track_id < 0 or float(lead_probs[index]) < WITNESS_PROB or urgent(lead):
        weight = 0.0
      else:
        target = tell_weight(object_statuses[index], distance_ratios[index])
        if track_id != self.track_ids[index]:
          weight = target
        else:
          previous = self.weights[index]
          weight = min(max(target, previous - release), previous + rise)
      self.track_ids[index] = track_id
      self.weights[index] = weight
    return list(self.weights)
