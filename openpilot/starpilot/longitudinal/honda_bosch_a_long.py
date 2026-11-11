"""Honda Bosch A Long Enhancement: longitudinal behavior that is active whenever the Bosch-A radar is on.

Ported from openpilot-radar (ns-bosch-radar-testing @ 567495f4ad + 6db22115e8, selfdrive/controls/lib/longitudinal_planner.py).
Active only when `active_for(CP)`: a Honda in VERIFIED_BOSCH_A_CARS with openpilot longitudinal and the radar not
disabled (`HondaBoschARadar`), which is the same gate radard uses to build BoschALeadPolicy. Each part also has its own
switch: Stock Brake Feel is permanent (owner 2026-11-11: "Make Stock Brake Feel permanent"; the BoschAStockBrakeFeel
toggle and its settings row are removed). Both longitudinal planners (StarPilot's and the upstream one) call it the
same way. The D-077 birth-rail ramps (off, never wired to radard here) were removed 2026-11-11; the code and its
evidence comment are in git at 3f26fb513c and in openpilot-radar radard.py @ 567495f4ad.

Contents:
- Stock Brake Feel (D-086, D-100, D-103, D-120): stock Honda ACC's TTC depth and jerk tables, with the soft exit,
  takeover hold, lead-stop law (<= 20 s) and the emergency latch.
- Brake onset ramp (2026-11-05, folded into step() 2026-11-11): a brake that starts with every lead far away deepens at
  most ONSET_JERK per second; with the law always on it only binds inside the emergency pass-through.
- Coast first (depth floor; folded in from bosch_a_coast_first.py 2026-11-11): for a distant lead tapping its brakes, the
  law's target is floored at COAST_ACCEL and the floor descends at COAST_RAMP_JERK. It runs last inside step(), on every
  return path, when the caller passes `follow_s`.

The planner calls `step()` once per cycle, after the other target owners and before release coast, smooth and stop gap.
Depth shaping (the law, the onset ramp, then the coast-first floor) is this one call. Every limit
here only shapes how a brake builds or lets off; the emergency latch passes the planner's target through untouched,
so the car never brakes harder than the planner would without this module. Replay and limited road evidence only.

HumanFollowing and HumanAcceleration are separate modules (human_following.py, human_acceleration.py) because they
also run with the radar off (vision-only).
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.longitudinal.bosch_a_toggles import vehicle_supported

# ---- Stock Brake Feel (D-086), comments abridged from the source; see STATUS/DECISIONS for the evidence -------------
# While a lead closes faster than STOCK_FEEL_MIN_CLOSING and the worst TTC is over STOCK_FEEL_TTC_FLOOR_S, the target may
# go no deeper than the depth table and may deepen no faster than the jerk table (fitted to stock ACC_CONTROL).
STOCK_FEEL_DEPTH_BP = [2.0, 2.25, 2.75, 3.5, 4.5, 5.5, 6.5, 7.5, 9.0, 11.0, 13.5, 17.5, 25.0]  # s
STOCK_FEEL_DEPTH_V = [-3.5, -3.2, -2.4, -2.3, -2.3, -2.1, -1.7, -1.6, -1.28, -1.09, -0.88, -0.74, -0.35]  # m/s^2
# Far entry (TTC >= 25 s): -0.6 instead of -0.35 while the nearest lead is within STOCK_FEEL_FAR_GAP_S of time gap
# (owner 2026-11-06). Route 96 32:17 (49 mph, lead then braking -3): the -0.35 cap held a plateau for ~6 s and the
# driver took over; 8f B3 slowed too late on the same plateau. An ungated -0.6 (c59ee88e4, driven on route 99) added
# far-lead brakes: vn80 -1.58 at gap 8.2 s, p78 -1.04 at gap 92 s. Only the time gap separates them: the plateau
# frames sit at 1.2-2.0 s (T4) and 3.3-3.6 s (B3), the far brakes at 9-10 s and 100+ s; lead decel, closing speed
# and a closing/distance need all overlapped (aLeadK reads -0.8..-1.2 on the far brakes). Replay (Job, open loop,
# Stock Brake Feel + Human Following + Approaching lead buffer on; limited road evidence): T4 min gap 0.54 -> 0.91 s,
# B3 2.26 -> 2.71 s, both equal to the ungated -0.6; 2.46 h pooled vs ungated: both far brakes gone, 3 marginal new
# <-1 episodes (two 1-frame -1.01/-1.03, one -1.18 at 7.8 s), all traced to closed-loop divergence.
STOCK_FEEL_FAR_DEPTH = -0.6  # m/s^2
STOCK_FEEL_FAR_GAP_S = 5.0   # s
STOCK_FEEL_JERK_BP = [2.0, 2.5, 6.0, 10.0]  # s
STOCK_FEEL_JERK_V = [3.0, 2.0, 1.0, 0.6]  # m/s^3
STOCK_FEEL_TTC_FLOOR_S = 2.0
STOCK_FEEL_MIN_CLOSING = 0.5  # m/s
# Outside the law (TTC at or under the floor, or no lead closing) the depth is kept but deepens no faster than this
# (STATUS 220: without it the target stepped -0.6..-1.65 in one frame; stock's own p98 is 2-3.4 m/s^3).
STOCK_FEEL_JERK_OUTSIDE = 5.0  # m/s^3
# D-125 (329 8:02): soft exit when the closing speed falls under STOCK_FEEL_MIN_CLOSING.
STOCK_FEEL_EXIT_JERK = 0.6   # m/s^3
STOCK_FEEL_EXIT_HOLD_S = 2.0  # s
# Takeover hold (owner 2026-10-08, 2f7 26:38): a brake already deeper than the cap is let off at this rate, not cut.
STOCK_FEEL_CAP_RELEASE_JERK = 0.3  # m/s^3
# Lead stopping (owner 2026-10-07, 2f2 1099/1483): a closing lead braking at STOCK_FEEL_LEAD_STOP_A or harder moves the
# cap to the decel needed to stop STOCK_FEEL_LEAD_STOP_GAP behind it. D-120 (324): leads more than
# STOCK_FEEL_LEAD_STOP_MAX_T s from stopping are left to the table (12 s delayed a real slowing lead on 324 2790).
STOCK_FEEL_LEAD_STOP_A = -1.0  # m/s^2
STOCK_FEEL_LEAD_STOP_GAP = 4.0  # m
STOCK_FEEL_LEAD_STOP_MAX_T = 20.0  # s
# Emergency bypass (owner 2026-10-07, 2f5 ~10:01: a lead braking -6.5 was met at stock's depth and the driver had to
# brake). Latched while any lead is closing: the planner's target passes through with no cap and no jerk limit.
STOCK_FEEL_EMERGENCY_A_LEAD = -2.0  # m/s^2
STOCK_FEEL_EMERGENCY_NEED = 2.0  # m/s^2
# The aLeadK path only latches when the worst TTC is at most this (openpilot-radar STATUS 258, 6db22115e8): 3a3
# 17:28.35 aLeadK -2.09 at TTC 21.7 s latched and stayed on through a separate build 15 s later that ran to -3.03.
STOCK_FEEL_EMERGENCY_A_LEAD_MAX_TTC = 10.0  # s


def active_for(CP) -> bool:
  """Same gate as radard's BoschALeadPolicy: verified Bosch-A Honda, radar on, openpilot longitudinal."""
  from opendbc.car.honda.bosch_a_radar import VERIFIED_BOSCH_A_CARS
  return bool(CP.brand == "honda" and CP.carFingerprint in VERIFIED_BOSCH_A_CARS and not CP.radarUnavailable and
              CP.openpilotLongitudinalControl)


def _present(lead) -> bool:
  return lead is not None and bool(getattr(lead, 'present', False))


def brake_onset_limited_target(prev: float, target: float, dt: float, jerk: float | None) -> float:
  """A brake below min(prev, 0) may deepen at most jerk * dt per step; throttle cuts, rises and jerk None pass through."""
  start = min(prev, 0.0)
  if jerk is None or target >= start:
    return float(target)
  return float(max(target, start - jerk * dt))


def brake_onset_ttc(leads, min_closing: float = 1e-3) -> float:
  """Worst time to contact (s) over the present closing leads; inf when none is closing."""
  ttc_min = float('inf')
  for lead in leads:
    if not _present(lead):
      continue
    closing = max(0.0, -float(lead.vRel))
    if closing > min_closing:
      ttc_min = min(ttc_min, float(lead.dRel) / closing)
  return ttc_min


def lead_stop_need(leads, v_ego: float, max_t: float = 0.0) -> float:
  """Decel (m/s^2, >= 0) to stop STOCK_FEEL_LEAD_STOP_GAP behind where the worst closing, braking lead will stop; 0 when
  no closing lead is braking at STOCK_FEEL_LEAD_STOP_A or harder. With max_t > 0, a lead more than max_t s from stopping
  is skipped."""
  need = 0.0
  for lead in leads:
    if not _present(lead) or not -float(lead.vRel) > STOCK_FEEL_MIN_CLOSING:
      continue
    a_lead = float(getattr(lead, 'aLeadK', 0.0))
    if not a_lead <= STOCK_FEEL_LEAD_STOP_A:
      continue
    v_lead = max(float(getattr(lead, 'vLead', v_ego + float(lead.vRel))), 0.0)
    if max_t > 0 and v_lead > max_t * -a_lead:
      continue
    room = float(lead.dRel) + v_lead * v_lead / (2.0 * -a_lead) - STOCK_FEEL_LEAD_STOP_GAP
    need = max(need, float(v_ego) ** 2 / (2.0 * max(room, 0.5)))
  return need


def stock_feel_emergency(leads, v_ego: float, active: bool) -> bool:
  """Latch: True while a lead is closing once a closing lead brakes at STOCK_FEEL_EMERGENCY_A_LEAD or harder, or stopping
  behind a braking lead needs at least STOCK_FEEL_EMERGENCY_NEED and more than the table's depth."""
  ttc = brake_onset_ttc(leads, STOCK_FEEL_MIN_CLOSING)
  if ttc == float('inf'):
    return False
  if active:
    return True
  for lead in leads:
    if _present(lead) and -float(lead.vRel) > STOCK_FEEL_MIN_CLOSING and float(lead.aLeadK) <= STOCK_FEEL_EMERGENCY_A_LEAD and \
       ttc <= STOCK_FEEL_EMERGENCY_A_LEAD_MAX_TTC:
      return True
  need = lead_stop_need(leads, v_ego)
  depth = stock_feel_depth(leads, ttc, v_ego)
  return need >= STOCK_FEEL_EMERGENCY_NEED and need > -depth


def stock_feel_held_cap(prev: float, cap: float, dt: float) -> float:
  """The cap, except that a brake already deeper than it (prev) is only let off at STOCK_FEEL_CAP_RELEASE_JERK."""
  if prev < cap:
    return float(min(cap, prev + STOCK_FEEL_CAP_RELEASE_JERK * dt))
  return float(cap)


def stock_feel_law_active(leads, emergency: bool) -> bool:
  """True while stock_feel_target applies the fitted law (a lead closing, TTC over the floor, no emergency)."""
  ttc = brake_onset_ttc(leads, STOCK_FEEL_MIN_CLOSING)
  return not emergency and ttc > STOCK_FEEL_TTC_FLOOR_S and ttc != float('inf')


def stock_feel_depth(leads, ttc: float, v_ego: float) -> float:
  """Depth table at this TTC; the far entry is STOCK_FEEL_FAR_DEPTH while the nearest lead is within
  STOCK_FEEL_FAR_GAP_S of time gap."""
  depth_v = STOCK_FEEL_DEPTH_V
  d_near = min((float(lead.dRel) for lead in leads if _present(lead)), default=float('inf'))
  if d_near <= STOCK_FEEL_FAR_GAP_S * max(float(v_ego), 0.0):
    depth_v = STOCK_FEEL_DEPTH_V[:-1] + [STOCK_FEEL_FAR_DEPTH]
  return float(np.interp(ttc, STOCK_FEEL_DEPTH_BP, depth_v))


def stock_feel_target(leads, prev: float, target: float, dt: float, v_ego: float = 0.0,
                      emergency: bool = False, law_recent: bool = False) -> float:
  """D-086 stock Honda ACC brake law (see the constants above)."""
  if emergency:
    return float(target)
  ttc = brake_onset_ttc(leads, STOCK_FEEL_MIN_CLOSING)
  if ttc == float('inf') and law_recent and not any(
      _present(lead) and float(lead.aLeadK) <= STOCK_FEEL_LEAD_STOP_A for lead in leads):
    return brake_onset_limited_target(prev, target, dt, STOCK_FEEL_EXIT_JERK)
  if not ttc > STOCK_FEEL_TTC_FLOOR_S or ttc == float('inf'):
    return brake_onset_limited_target(prev, target, dt, STOCK_FEEL_JERK_OUTSIDE)
  depth = stock_feel_depth(leads, ttc, v_ego)
  need = lead_stop_need(leads, v_ego, STOCK_FEEL_LEAD_STOP_MAX_T)
  if need > -depth:
    return brake_onset_limited_target(prev, max(target, stock_feel_held_cap(prev, -min(need, 3.5), dt)), dt,
                                      STOCK_FEEL_JERK_OUTSIDE)
  target = max(target, stock_feel_held_cap(prev, depth, dt))
  return brake_onset_limited_target(prev, target, dt, float(np.interp(ttc, STOCK_FEEL_JERK_BP, STOCK_FEEL_JERK_V)))


# ---- Brake onset ramp --------------------------------------------------------------------------------------------
# Brake onset ramp: a brake that starts with every lead far away builds at ONSET_JERK instead of in one frame.
#
# Owner 2026-11-05: "Why can we just mimic stock brake for those situations? Instead of just slamming the brake".
# Stock ACC on route 299 (STATUS 214) brings a brake in at -1.4..-3.4 m/s^3 per step; ours stepped at -36..-52 m/s^3
# on the route 84 far brakes (E1, E4), e2ae and e2f8. The old D-083 ramp (6f1a05b37) would not have changed them: its
# lead-braking and need gates stand it down on every one, because the lead is already braking hard at onset.
#
# Replay (Job, base dad6bbbb6, open loop, arm R4b; limited road evidence, not driven):
#   - first-jolt jerk 44-52 -> 8 m/s^3 on 84 E1/E4/vn84_3/vn84_11, 34 -> 8 on e2ae, 43 -> 8 on e2f8; peak decel unchanged
#     (-3.5 stays -3.5), onset at -2 at most 0.15 s later.
#   - min gap 0.4-0.9 m smaller on those events (e2ae: -0.62 m, ending at 4.8 m); min TTC within 0.08 s.
#   - the 2f5 close stop (onset dRel 38.1 m) is exempt and equals today's output; pooled 0.85 h: same brake episode
#     counts, no episode closer by more than 2 m.
#   - nothing at onset (need, closing speed) separates e2ae/2f5 from the 84 far brakes; only the onset distance does.
#     EXEMPT_D_M = 40 sits between 2f5 (38.1 m) and e2f8 (43.9 m); 30 m capped 2f5 and cost it 0.36 m.
#   - an in-brake close-gap stand-down (R5) changed nothing: the ramp finishes in ~0.4 s, before the gap gets close.
#
# Channels: reads radarState leadOne/leadTwo (present, dRel, vRel) as published; a railed vRel (U11 low rail -12) only
# makes TTC smaller, which stands the ramp down, never extends it. Only the fall rate of the target is capped; the peak is
# never reduced, rises pass through, and dRel/vLead/radar points are untouched (D-041/D-042).
#
# With the law always on (owner 2026-11-11), the ramp only binds inside the emergency pass-through, where the law
# hands the planner's target through untouched; on the scorecard baseline (44f382face, 36 routes) that was 10 frames in
# 7 episodes (replay). The episode-end rule below (the law's output and last cycle's published target both back at
# or above END_ACCEL) replaces the old ramp's own post-smooth input test: static; not replayed yet.
ONSET_JERK = 8.0         # m/s^3, the fastest a non-exempt brake may deepen
ONSET_START_ACCEL = -0.3       # m/s^2: the law's output below this starts a brake episode
ONSET_END_ACCEL = -0.1         # m/s^2: the output and last cycle's target both back at or above this end it
ONSET_EXEMPT_D_M = 40.0        # any lead closer than this as the episode starts: no ramp for the whole episode
ONSET_TTC_STAND_DOWN_S = 2.0   # any lead at or below this TTC: no ramp for the rest of the episode


def lead_close(leads) -> bool:
  return any(float(lead.dRel) < ONSET_EXEMPT_D_M for lead in leads if _present(lead))


def lead_ttc_low(leads) -> bool:
  for lead in leads:
    if not _present(lead):
      continue
    closing = -float(lead.vRel)
    if closing > 1e-3 and float(lead.dRel) / closing <= ONSET_TTC_STAND_DOWN_S:
      return True
  return False


# ---- Coast first (depth floor) ------------------------------------------------------------------------------------
# Bosch-A coast first: for a distant lead tapping its brakes, lift off the gas before braking.
#
# Owner 2026-11-01, route 71 (00000071--37216af48f) 12:03: at 70 mph a pickup 51-66 m ahead (1.6-2.0 s) braked for
# about 2 s (aLeadK to -3.0, vLead 32.5 -> 25.4) and was back on the gas 1.5 s later. The planner went 0 -> -2.25 in
# 0.75 s. The closest gap was 50.9 m / 1.64 s. A coast at -0.5 from the same onset reaches about 49.5 m / 1.65 s
# (open-loop kinematics on the logged lead, Long Sim), so the brake bought about 1.4 m. "I wouldn't mind if the car lets
# go off the gas instead, then ramping up brake if needed" (owner 2026-10-20, 3a3 17:44).
#
# While the gap is well clear, the brake is staged instead of cut: the target is floored at COAST_ACCEL when braking
# starts and the floor descends at COAST_RAMP_JERK, so a lead that keeps braking still gets the full brake 1-2 s later.
# Every tick re-checks a projection: the lead keeps its current decel (aLeadK, no recovery assumed) and we follow the
# staged floor for COAST_HORIZON_S; if the projected gap ever drops under COAST_FLOOR_GAP_S * v + COAST_FLOOR_GAP_M, or
# the time gap is under follow + COAST_GAP_MARGIN_S, or TTC, lead decel or FCW trip, the floor is dropped at once and the
# planner's target passes untouched until it stops braking. It only raises a braking target (never adds gas above
# COAST_ACCEL, never touches the radar points: D-041/D-042). Static and open-loop replay evidence only.
#
# Owner 2026-11-05, route 0000008f bookmark 1 (4445.2): a car 37-40 m ahead (2.7 s, TTC 14.5 s) tapped its brakes at
# about -3 for 1 s at 32 mph and we braked to -1.8. Coast first stood down on speed (14.1 < 15) and on the projection,
# which held the lead's -3 for the full 3 s. "The car was still a distance away". So COAST_MIN_V_EGO is 13.5 and the
# projection holds the lead's decel for COAST_LEAD_DECEL_HOLD_S, then holds the lead's speed. Replay (Long Sim, V-ASM
# 3b3d20056 with Stock Brake Feel and Human Following on, open loop on the logged lead): bookmark 1 coasts 1.75 s, peak
# -1.81 -> -1.58, min gap unchanged (26.9 m); bookmark 2 (1.43 s gap) unchanged; 2f5 one frame later, e2ae and route 84
# E1/E4 identical, 19 route-84 far brakes keep min gap and TTC; route 71 12:03 identical. Pooled 2.16 h: 14 new coasts
# (37.1 -> 43.5/h), none lost, counts below -1 and -2 identical, max target change 0.18 m/s^2, closest new coast 2.29 s
# gap / 9.9 s TTC. Replay only, not driven.
COAST_MIN_V_EGO = 13.5       # m/s (~30 mph); distant taps at road speed (was 15; 8f B1 at 14.1). The stop gap owns the stop.
COAST_ACCEL = -0.5           # m/s^2; lift-off plus a little (Civic gas-off at 70 mph ~ -0.4)
COAST_RAMP_JERK = 1.0        # m/s^3; staged floor: -0.5 -> -2.5 in 2 s
COAST_GAP_MARGIN_S = 0.35    # time gap must be at least follow + this (aggressive 1.25 -> 1.6; 71 12:03 bottomed at 1.64)
COAST_HORIZON_S = 3.0
COAST_LEAD_DECEL_HOLD_S = 1.5  # s; the projected lead brakes this long, then holds its speed (was the whole horizon)
COAST_FLOOR_GAP_S = 0.8      # projected gap floor: 0.8 s * v + 6 m (long_mpc STOP_DISTANCE)
COAST_FLOOR_GAP_M = 6.0
COAST_MIN_TTC_S = 5.0        # 71 12:03 at the trough: 51 m / 5.8 m/s = 8.8 s
COAST_MAX_LEAD_DECEL = -4.0  # m/s^2; harder than this is not a tap


def projected_gap_ok(d_rel: float, v_lead: float, a_lead: float, v_ego: float, floor_a: float) -> bool:
  a_lead = min(a_lead, 0.0)
  d, vl, ve, fa = d_rel, v_lead, v_ego, floor_a
  t = 0.0
  while t < COAST_HORIZON_S:
    d += (vl - ve) * DT_MDL
    if t < COAST_LEAD_DECEL_HOLD_S:
      vl = max(0.0, vl + a_lead * DT_MDL)
    ve = max(0.0, ve + fa * DT_MDL)
    fa -= COAST_RAMP_JERK * DT_MDL
    if d < COAST_FLOOR_GAP_S * ve + COAST_FLOOR_GAP_M:
      return False
    t += DT_MDL
  return True


class HondaBoschALongEnhancement:
  def __init__(self, CP, dt: float):
    """Stock Brake Feel is permanent (owner 2026-11-11); active_for(CP) is the only gate."""
    active = active_for(CP)
    self.stock_feel = active
    # Implausible-drop bound (D-041/D-042; owner bookmark route a2 seg 50, 2026-11-07): same gate as the other active
    # checks in this module (Bosch-A supported, radar available); default ON, no toggle.
    self.drop_bound = ImplausibleDropBound() if active else None
    self.dt = dt
    # Coast first: same gate as the old BoschACoastFirst.enabled (not active_for / stock_feel).
    self.coast_enabled = vehicle_supported(CP) and not bool(getattr(CP, 'radarUnavailable', True))
    self.coast_floor = None      # staged floor while coasting first; None when not active (replay/debug)
    self.coast_bypassed = False  # latched for the rest of this braking event
    self.reset()

  def planner_radar_state(self, radar_state, a_ego: float, model_v2=None):
    """The radarState the MPC sees: the implausible-drop bound (below) clamps a leadOne velocity-glitch brake;
    otherwise radar_state itself."""
    if self.drop_bound is not None:
      radar_state = bound_implausible_drop(radar_state, model_v2, self.drop_bound)
    return radar_state

  def reset(self):
    self._reset_law()
    self.onset_episode = False
    self.onset_free = False  # this episode passes through (close lead at its start, low TTC, FCW or stopping)

  def _reset_law(self):
    self.emergency = False
    self.law_t = 0.0

  def reset_coast(self):
    self.coast_floor = None
    self.coast_bypassed = False

  @property
  def coast_active(self) -> bool:
    return self.coast_floor is not None

  def step(self, *, lead_one, lead_two, prev: float, target: float, v_ego: float,
           stopping: bool, standstill: bool, reset_state: bool, fcw: bool, follow_s: float | None = None) -> float:
    """Shape the final acceleration target: the law and onset ramp, then the coast-first floor on every return path.
    `stopping` covers should-stop, force stop and red-light stops. `prev` is last cycle's published target; `target` is
    this cycle's before the law and the onset ramp. `follow_s` is the planner's follow time; None skips coast first
    (the upstream planner never ran it)."""
    out = self._shape(lead_one=lead_one, lead_two=lead_two, prev=prev, target=target, v_ego=v_ego,
                      stopping=stopping, standstill=standstill, reset_state=reset_state, fcw=fcw)
    if follow_s is None:
      return out
    return self._coast_first(lead_one=lead_one, lead_two=lead_two, target=out, v_ego=v_ego, follow_s=follow_s,
                             fcw=fcw, reset_state=reset_state)

  def _coast_first(self, *, lead_one, lead_two, target: float, v_ego: float, follow_s: float, fcw: bool,
                   reset_state: bool) -> float:
    target = float(target)
    if not self.coast_enabled or reset_state:
      self.reset_coast()
      return target
    if target >= COAST_ACCEL:
      self.reset_coast()  # not braking past a lift-off: the event is over
      return target
    if self.coast_bypassed:
      return target
    from openpilot.starpilot.longitudinal.bosch_a_stop_gap import closest_radar_lead
    lead = closest_radar_lead((lead_one, lead_two))
    floor = COAST_ACCEL if self.coast_floor is None else self.coast_floor - COAST_RAMP_JERK * self.dt
    if lead is None or fcw or v_ego < COAST_MIN_V_EGO or not self._clear(lead, v_ego, follow_s, floor):
      self.coast_floor = None
      self.coast_bypassed = True
      return target
    self.coast_floor = floor
    return max(target, floor)

  @staticmethod
  def _clear(lead, v_ego: float, follow_s: float, floor: float) -> bool:
    d_rel, v_lead, a_lead = float(lead.dRel), float(lead.vLead), float(getattr(lead, 'aLeadK', 0.0))
    closing = v_ego - v_lead
    return (d_rel >= (follow_s + COAST_GAP_MARGIN_S) * v_ego and
            (closing <= 0.0 or d_rel / closing >= COAST_MIN_TTC_S) and
            a_lead >= COAST_MAX_LEAD_DECEL and
            projected_gap_ok(d_rel, v_lead, a_lead, v_ego, floor))

  def _shape(self, *, lead_one, lead_two, prev: float, target: float, v_ego: float,
             stopping: bool, standstill: bool, reset_state: bool, fcw: bool) -> float:
    """The Stock Brake Feel law plus the brake onset ramp (before coast first)."""
    if not self.stock_feel or reset_state or standstill:
      self.reset()
      return float(target)
    leads = (lead_one, lead_two)
    out = float(target)
    if stopping:
      self._reset_law()
    else:
      self.emergency = stock_feel_emergency(leads, v_ego, self.emergency)
      out = stock_feel_target(leads, prev, target, self.dt, v_ego, self.emergency, self.law_t > 0.0)
      self.law_t = STOCK_FEEL_EXIT_HOLD_S if stock_feel_law_active(leads, self.emergency) else max(0.0, self.law_t - self.dt)
    if self.onset_episode and out >= ONSET_END_ACCEL and prev >= ONSET_END_ACCEL:
      self.onset_episode = False
      self.onset_free = False
    if not self.onset_episode and out < ONSET_START_ACCEL:
      self.onset_episode = True
      self.onset_free = lead_close(leads)
    if self.onset_episode and (fcw or stopping or lead_ttc_low(leads)):
      self.onset_free = True
    if self.onset_episode and not self.onset_free:
      out = brake_onset_limited_target(prev, out, self.dt, ONSET_JERK)
    return float(out)


class _BoundedRadarState:
  def __init__(self, radar_state, lead_one, lead_two):
    self._radar_state = radar_state
    self.leadOne = lead_one
    self.leadTwo = lead_two

  def __getattr__(self, name):
    return getattr(self._radar_state, name)


# ---- Implausible radar-drop bound on leadOne (owner bookmark, route 000000a2--794ebefbf7 seg 50, 2026-11-07) --------
# Right after a left lane change, leadOne was radar track 24 at 18 m, matched to the camera lead (leadsV3[0] x~19-20 m,
# prob 1.0). Over 0.7 s track 24's vLead collapsed 12.5 -> 2.9 m/s (vRel +1.1 -> -6.9, an implied -13 m/s^2, physically
# impossible), aLeadK pinned at -3.0. The camera lead's v stayed 10.5-11.9 m/s and x 19-20 m the whole time. The planner
# braked at -3.5 and the car went 10.3 -> 4.2 m/s; the track then died and the next track (62) showed the car pulling
# away at 22 m. A radar velocity glitch, not a real lead. D-041/D-042: the point is never dropped or altered in
# radarState; this only bounds what leadOne publishes to the planner (MPC + this module's stock-feel law). Replay only
# (16-drive census + 12-episode gate rerun, 2026-11-07); no road evidence yet for this specific bound.
# Residual risk: a real lead braking harder than DROP_MPS/DROP_WINDOW_S (>10 m/s^2 - itself a crash-decel lead) while
# the camera model lags behind gets up to MAX_S = 1.0 s of the camera's (slower-braking) speed instead of the radar's.
DROP_WINDOW_S = 0.5          # s; a2 seg 50: the glitch collapsed vLead inside this window
DROP_MPS = 5.0               # m/s over DROP_WINDOW_S; 5.0 / 0.5 s = 10 m/s^2, beyond a real car's braking
MODEL_DROP_MAX_MPS = 1.5     # m/s the camera lead's v may drop in the window and still call the radar drop a glitch
# Range-split census, 2026-11-07, replay. Model x change over the 0.5 s before the trigger, in m: a2 seg 50 phantom
# +0.50; 84 s15 radar-noise legs +2.44 and -3.81; real slowdowns 7b s27 -5.84, 99 s1 -5.05, 7b s23 -5.81, 78 s3 -5.19,
# 7b s8 -3.69, 80 s6 -8.81, 80 s19 -1.06; adjacent-lane truck 8a s2 -2.92; 8a s11 (disengaged, lead steady) -0.09.
# 0.3 m sits between a2 (+0.50) and the closest real slowdown, 80 s19 (-1.06). A magnitude gate on radar-vs-camera
# range split was tried first and rejected: 7b s23 split -8.3 m is deeper than a2's -3.4 m. Not firing falls back to
# the old behaviour (the radar drop is published unbounded), so the gate only narrows the bound.
MODEL_RANGE_SHRINK_MAX_M = 0.3  # m the camera lead's x may shrink over DROP_WINDOW_S and still call the radar drop a glitch
AGREE_M = 4.0                # m; radar dRel vs (model x - RADAR_TO_CAMERA) agreement needed at the window start
MODEL_PROB_MIN = 0.9         # camera lead confidence needed to trust it over the radar
TRIGGER_V_EGO_MIN = 5.0      # m/s; below this the radar's own near-field behavior is not glitch-like, stay off
END_CONVERGE_MPS = 2.0       # m/s; |radar vLead - model v| under this: radar has caught back up, end the bound
END_MODEL_DROP_MPS = 3.0     # m/s the camera lead's v may fall since the trigger before releasing immediately
MAX_S = 1.0                  # s; hard cap on how long the bound can run
RETRIGGER_COOLDOWN_S = 2.0   # s a track id is blocked from re-triggering after the bound ends on it
MODEL_V_MARGIN_MPS = 1.0     # m/s below the model's v that the bounded vLead may not go
IMPLAUSIBLE_DROP_DT = 0.05   # s per planner frame (DT_MDL)


# Lazy import, same reasoning as active_for's: this module is imported before radard's car gates are known to apply.
def _radar_to_camera() -> float:
  from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
  return RADAR_TO_CAMERA


def _model_lead0(model_v2):
  """(prob, x[0], v[0], a[0]) of the first model lead, or None when modelV2/leadsV3[0] is unusable."""
  if model_v2 is None:
    return None
  try:
    leads = model_v2.leadsV3
    if len(leads) < 1:
      return None
    lead0 = leads[0]
    return float(lead0.prob), float(lead0.x[0]), float(lead0.v[0]), float(lead0.a[0])
  except (AttributeError, IndexError, TypeError, ValueError):
    return None


def _drop_radar_lead(lead) -> bool:
  return _present(lead) and bool(getattr(lead, 'radar', False)) and int(getattr(lead, 'radarTrackId', -1)) >= 0


class ImplausibleDropBound:
  """Per-leadOne state for the implausible-drop bound (see the constants above). `bound_implausible_drop` drives it
  once per planner frame."""

  def __init__(self):
    self.frame = 0
    self.track_id: int | None = None
    self.hist: list[tuple[int, float, float, tuple | None]] = []  # (frame, vLead, dRel, model) trimmed to the
    # trailing DROP_WINDOW_S of the current track; model is (prob, x, v, a) or None
    self.active = False
    self.trigger_frame = 0
    self.trigger_model_v = 0.0
    self.cooldown_until: dict[int, int] = {}

  def tick(self):
    self.frame += 1

  def _end(self, track_id: int):
    self.active = False
    self.cooldown_until[track_id] = self.frame + round(RETRIGGER_COOLDOWN_S / IMPLAUSIBLE_DROP_DT)

  def _reset_track(self, track_id: int | None):
    if self.active and self.track_id is not None:
      self._end(self.track_id)
    self.track_id = track_id
    self.hist = []

  def update(self, lead, model_v2):
    """`lead` is radarState.leadOne this frame. Returns (vLead, vRel, aLeadK) to publish, or None to leave it as is."""
    if not _drop_radar_lead(lead):
      self._reset_track(None)
      return None
    track_id = int(lead.radarTrackId)
    if track_id != self.track_id:
      self._reset_track(track_id)
    model = _model_lead0(model_v2)
    v_lead = float(lead.vLead)
    d_rel = float(lead.dRel)
    v_ego = v_lead - float(lead.vRel)
    self.hist.append((self.frame, v_lead, d_rel, model))
    while self.hist and (self.frame - self.hist[0][0]) * IMPLAUSIBLE_DROP_DT > DROP_WINDOW_S:
      self.hist.pop(0)

    if self.active:
      if model is None or abs(v_lead - model[2]) < END_CONVERGE_MPS or \
         (self.trigger_model_v - model[2]) > END_MODEL_DROP_MPS or \
         (self.frame - self.trigger_frame) * IMPLAUSIBLE_DROP_DT >= MAX_S:
        self._end(track_id)
        return None
      return self._bounded(lead, model, v_ego)

    if model is None or model[0] <= MODEL_PROB_MIN or v_ego <= TRIGGER_V_EGO_MIN or len(self.hist) < 2 or \
       self.cooldown_until.get(track_id, -1) > self.frame:
      return None
    start_frame, start_v_lead, start_d_rel, start_model = self.hist[0]
    if start_model is None:
      return None
    max_v_lead = max(h[1] for h in self.hist)
    if not max_v_lead - v_lead > DROP_MPS:
      return None
    if not start_model[2] - model[2] < MODEL_DROP_MAX_MPS:
      return None
    if not model[1] - start_model[1] > -MODEL_RANGE_SHRINK_MAX_M:
      return None
    if not abs(start_d_rel - (start_model[1] - _radar_to_camera())) < AGREE_M:
      return None
    self.active = True
    self.trigger_frame = self.frame
    self.trigger_model_v = model[2]
    return self._bounded(lead, model, v_ego)

  @staticmethod
  def _bounded(lead, model: tuple, v_ego: float):
    """vLead/vRel/aLeadK bounded only toward a faster, less-braking lead; dRel is untouched (D-041/D-042)."""
    model_v, model_a = model[2], model[3]
    v_lead_b = max(float(lead.vLead), model_v - MODEL_V_MARGIN_MPS)
    v_rel_b = v_lead_b - v_ego
    a_lead_k_b = max(float(lead.aLeadK), min(model_a, 0.0))
    return v_lead_b, v_rel_b, a_lead_k_b


class _FieldBoundedLead:
  """A lead that reads as `lead` except for the given field overrides."""
  def __init__(self, lead, **fields):
    self._lead = lead
    for name, value in fields.items():
      setattr(self, name, value)
    self._fields = fields

  def __getattr__(self, name):
    return getattr(self._lead, name)


def bound_implausible_drop(radar_state, model_v2, state: ImplausibleDropBound):
  state.tick()
  result = state.update(radar_state.leadOne, model_v2)
  if result is None:
    return radar_state
  v_lead_b, v_rel_b, a_lead_k_b = result
  fields = {'vLead': v_lead_b, 'vLeadK': v_lead_b, 'vRel': v_rel_b, 'aLeadK': a_lead_k_b}
  lead = radar_state.leadOne
  if hasattr(lead, 'aLead'):
    fields['aLead'] = a_lead_k_b
  bounded = _FieldBoundedLead(lead, **fields)
  return _BoundedRadarState(radar_state, bounded, radar_state.leadTwo)
