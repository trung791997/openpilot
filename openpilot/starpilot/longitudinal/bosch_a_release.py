"""Bosch-A release shaper: release coast (no gas off a departing radar lead), then the brake release limit and dwell; plus the short plan read-ahead.

Section 1, release coast:

Bosch-A release coast: a lift instead of gas when a radar lead that was braking in our lane leaves it.

Owner bookmark, route 9e (replay evidence), 2026-11-07: following a car slowing to turn right, radar track 39 is
in our lane at 21 m, closing at 7 m/s, and openpilot is braking. As the car turns off, the radar lead is released at
-5.16 s (it is 1.24 m off centre). leadOne then becomes a camera lead 84-96 m away, the planner's accel target jumps
from braking to about +0.5, and the car asks for gas while the turning car is still partly in front of us. "I'd
rather it lift instead of adding gas" (owner, route 9e, 2026-11-07).

The earlier design kept a 1.8 m lateral path box around the departing track instead of reading the radar handover
directly. Census (2026-11-07) found it also re-kept a box truck in the next lane at -3.0 and cars riding the path
bend through ordinary turns, so it is not used here: the release coast only withholds gas off a release it can see
directly in radarState, and it never adds braking of its own.

Trigger (checked every planner cycle, previous leadOne vs current leadOne): the previous cycle's leadOne was a
present, radar-matched lead (radarTrackId >= 0) closing faster than RELEASE_VREL_MAX inside RELEASE_D_MAX, and in the
current cycle either leadOne is gone (not present) or it is a different track that jumped more than RELEASE_JUMP_M
farther away. For HOLD_S after that, the output accel is capped at 0 - it only removes positive accel, so a target
that is already braking passes through unchanged. The hold ends early if a lead within RELEASE_D_MAX that is closing
appears (the cap is harmless there, since the planner would already be braking for it). The trigger itself is
skipped during a lane change (modelV2.meta.laneChangeState != off) or with a turn signal on (carState.leftBlinker or
rightBlinker), since passing a slower car also releases its track and we want to accelerate there, and skipped while
the gas is pressed. Static/replay-evidence only, not driven.


Section 2, brake release smoothing and read-ahead:

Bosch-A brake release smoothing and short plan read-ahead (always on for Bosch-A Hondas, no toggle).

Ported from openpilot-radar ns-bosch-radar-testing, where these were tuned against mvl's sp-honda-dev planner:
  ffe967495d  brake release limit: while braking, the target rises at most BRAKE_RELEASE_JERK per second.
  927f430189  brake release dwell: a brake is held BRAKE_RELEASE_DWELL_TICKS cycles before it may rise.
  b03716e918  short plan read-ahead: the MPC plan is read PLANNER_ACTION_T_S ahead instead of delay + DT_MDL.
Not ported: 75f27bd6cd (coast-ceiling slew). It smoothed an output clip that stepped when the throttle gate flipped.
Neither planner here has that clip; the gate only limits a_cruise, which get_cruise_accel already slews.

Evidence: ns replay only (16 log windows, jerk RMS 0.82x, brake onset unchanged by the release steps). The read-ahead
cost there was the hardest brake onset up to 0.3 s later and closed-loop min gap 6.0 -> 5.6 m. These numbers came from
a different planner. Here the evidence is static and unit tests only.

"""
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.longitudinal.bosch_a_toggles import vehicle_supported

RELEASE_VREL_MAX = -2.0  # m/s; previous lead must be closing at least this fast (9e: -7 m/s at release)
RELEASE_D_MAX = 40.0     # m; previous lead must be inside this (9e: 21 m at release)
RELEASE_JUMP_M = 20.0    # m; a handover lead farther than prev dRel + this counts as the same release (9e: 21 -> 84+)
HOLD_S = 1.0             # s; owner asked for "about a second" before resuming (route 9e, 2026-11-07)
BRAKE_RELEASE_JERK = 2.5  # m/s^3
BRAKE_RELEASE_DWELL_TICKS = 2  # 0.1 s at DT_MDL
PLANNER_ACTION_T_S = 0.30  # s; never longer than the car's own delay + DT_MDL

LaneChangeState = log.LaneChangeState


def _closing_radar_lead(prev) -> bool:
  return (prev is not None and bool(getattr(prev, 'present', False)) and
          int(getattr(prev, 'radarTrackId', -1)) >= 0 and
          float(prev.vRel) < RELEASE_VREL_MAX and float(prev.dRel) < RELEASE_D_MAX)


def _released(prev, cur) -> bool:
  if not _closing_radar_lead(prev):
    return False
  if cur is None or not bool(getattr(cur, 'present', False)):
    return True
  return (int(getattr(cur, 'radarTrackId', -1)) != int(getattr(prev, 'radarTrackId', -1)) and
          float(cur.dRel) > float(prev.dRel) + RELEASE_JUMP_M)


def _close_closing_lead(lead) -> bool:
  return (lead is not None and bool(getattr(lead, 'present', False)) and
          float(lead.dRel) < RELEASE_D_MAX and float(lead.vRel) < 0.0)


def _snapshot(lead):
  """Copy the fields we compare, so the previous cycle never aliases a reused or rebuilt radarState lead."""
  if lead is None:
    return None
  return SimpleNamespace(present=bool(getattr(lead, 'present', False)), radarTrackId=int(getattr(lead, 'radarTrackId', -1)),
                         dRel=float(lead.dRel), vRel=float(lead.vRel))


def brake_release_limited_target(prev: float, target: float, dt: float) -> float:
  """While braking (prev < 0), the target may rise at most BRAKE_RELEASE_JERK * dt per step; it may always fall."""
  if prev >= 0.0:
    return float(target)
  return float(min(target, prev + BRAKE_RELEASE_JERK * dt))


def brake_release_dwell_target(prev: float, target: float, rise_ticks: int) -> tuple[float, int]:
  """While braking, a rise is held for the first BRAKE_RELEASE_DWELL_TICKS cycles that ask for it.
  Returns (target, rise_ticks). It only ever holds more braking, so it cannot delay brake onset."""
  if target <= prev + 1e-3:
    return float(target), 0
  rise_ticks += 1
  if prev < 0.0 and rise_ticks <= BRAKE_RELEASE_DWELL_TICKS:
    return float(prev), rise_ticks
  return float(target), rise_ticks


def planner_action_t(actuator_delay: float, enabled: bool) -> float:
  default = actuator_delay + DT_MDL
  return min(PLANNER_ACTION_T_S, default) if enabled else default


class BoschARelease:
  def __init__(self, CP, dt: float = DT_MDL):
    self.enabled = vehicle_supported(CP)
    self.coast_enabled = vehicle_supported(CP) and not bool(getattr(CP, 'radarUnavailable', True))
    self.dt = dt
    self.hold_s = 0.0
    self.prev_lead = None  # leadOne from the previous cycle
    self.rise_ticks = BRAKE_RELEASE_DWELL_TICKS + 1

  def action_t(self, actuator_delay: float) -> float:
    return planner_action_t(actuator_delay, self.enabled)

  def reset_coast(self):
    self.hold_s = 0.0
    self.prev_lead = None

  def coast(self, *, lead_one, target: float, lane_change_state=None, left_blinker: bool = False,
            right_blinker: bool = False, gas_pressed: bool = False, reset_state: bool = False) -> float:
    target = float(target)
    if not self.coast_enabled or reset_state:
      self.reset_coast()
      return target
    prev_lead, self.prev_lead = self.prev_lead, _snapshot(lead_one)

    skip = (lane_change_state is not None and lane_change_state != LaneChangeState.off) or left_blinker or right_blinker or gas_pressed
    if self.hold_s > 0.0:
      if skip or _close_closing_lead(lead_one):
        self.hold_s = 0.0  # a closing lead is back; the cap is harmless but the hold is done
      else:
        self.hold_s = max(0.0, self.hold_s - self.dt)

    if self.hold_s <= 0.0:
      if not skip and _released(prev_lead, lead_one):
        self.hold_s = HOLD_S

    return min(target, 0.0) if self.hold_s > 0.0 else target

  def smooth(self, prev: float, target: float, reset: bool) -> float:
    """Last shaper before the final clip. `prev` is last cycle's published target. `reset` covers disengaged and standstill."""
    if not self.enabled or reset:
      self.rise_ticks = BRAKE_RELEASE_DWELL_TICKS + 1
      return float(target)
    target = brake_release_limited_target(prev, target, self.dt)
    target, self.rise_ticks = brake_release_dwell_target(prev, target, self.rise_ticks)
    return target

  def step(self, *, lead_one, prev: float, target: float, lane_change_state=None, left_blinker: bool = False,
           right_blinker: bool = False, gas_pressed: bool = False, reset_state: bool = False,
           standstill: bool = False) -> float:
    target = self.coast(lead_one=lead_one, target=target, lane_change_state=lane_change_state, left_blinker=left_blinker,
                        right_blinker=right_blinker, gas_pressed=gas_pressed, reset_state=reset_state)
    return self.smooth(prev, target, reset=bool(reset_state or standstill))
