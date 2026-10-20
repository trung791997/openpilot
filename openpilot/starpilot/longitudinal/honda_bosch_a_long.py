"""Honda Bosch A Long Enhancement: longitudinal behaviour that is active whenever the Bosch-A radar is on.

Ported from openpilot-radar (ns-bosch-radar-testing @ 567495f4ad, selfdrive/controls/lib/longitudinal_planner.py).
Active only when `active_for(CP)`: a Honda in VERIFIED_BOSCH_A_CARS with openpilot longitudinal and the radar not
disabled (`HondaBoschARadar`), which is the same gate radard uses to build BoschALeadPolicy. There is no other toggle.

Contents:
- Stock Brake Feel (D-086, D-100, D-103, D-120): stock Honda ACC's TTC depth and jerk tables, with the soft exit,
  takeover hold, lead-stop law (<= 20 s) and the emergency latch.
- Stop ease (planner side): the brake eases toward about -0.5 m/s^2 at the stop when there is room.

The planner calls `step()` once per cycle, after every other target owner and before the final clip. Every limit
here only shapes how a brake builds or lets off; the emergency latch passes the planner's target through untouched,
so the car never brakes harder than the planner would without this module. Replay and limited road evidence only.

HumanFollowing and HumanAcceleration are separate modules (human_following.py, human_acceleration.py) because they
also run with the radar off (vision-only).
"""
import numpy as np

# ---- Stock Brake Feel (D-086), comments abridged from the source; see STATUS/DECISIONS for the evidence -------------
# While a lead closes faster than STOCK_FEEL_MIN_CLOSING and the worst TTC is over STOCK_FEEL_TTC_FLOOR_S, the target may
# go no deeper than the depth table and may deepen no faster than the jerk table (fitted to stock ACC_CONTROL).
STOCK_FEEL_DEPTH_BP = [2.0, 2.25, 2.75, 3.5, 4.5, 5.5, 6.5, 7.5, 9.0, 11.0, 13.5, 17.5, 25.0]  # s
STOCK_FEEL_DEPTH_V = [-3.5, -3.2, -2.4, -2.3, -2.3, -2.1, -1.7, -1.6, -1.28, -1.09, -0.88, -0.74, -0.35]  # m/s^2
STOCK_FEEL_JERK_BP = [2.0, 2.5, 6.0, 10.0]  # s
STOCK_FEEL_JERK_V = [3.0, 2.0, 1.0, 0.6]  # m/s^3
STOCK_FEEL_TTC_FLOOR_S = 2.0
STOCK_FEEL_MIN_CLOSING = 0.5  # m/s
# Outside the law (TTC at or under the floor, or no lead closing) the depth is kept but deepens no faster than this
# (STATUS 220: without it the target stepped -0.6..-1.65 in one frame; stock's own p98 is 2-3.4 m/s^3).
STOCK_FEEL_JERK_OUTSIDE = 5.0  # m/s^3
# D-125 (329 8:02): soft exit when the closing speed falls under STOCK_FEEL_MIN_CLOSING.
STOCK_FEEL_SOFT_EXIT = True
STOCK_FEEL_EXIT_JERK = 0.6   # m/s^3
STOCK_FEEL_EXIT_HOLD_S = 2.0  # s
# Takeover hold (owner 2026-10-08, 2f7 26:38): a brake already deeper than the cap is let off at this rate, not cut.
STOCK_FEEL_CAP_RELEASE_JERK = 0.3  # m/s^3
# Lead stopping (owner 2026-10-07, 2f2 1099/1483): a closing lead braking at STOCK_FEEL_LEAD_STOP_A or harder moves the
# cap to the decel needed to stop STOCK_FEEL_LEAD_STOP_GAP behind it. D-120 (324): leads more than
# STOCK_FEEL_LEAD_STOP_MAX_T s from stopping are left to the table (12 s delayed a real slowing lead on 324 2790).
STOCK_FEEL_LEAD_STOP = True
STOCK_FEEL_LEAD_STOP_A = -1.0  # m/s^2
STOCK_FEEL_LEAD_STOP_GAP = 4.0  # m
STOCK_FEEL_LEAD_STOP_MAX_T = 20.0  # s
# Emergency bypass (owner 2026-10-07, 2f5 ~10:01: a lead braking -6.5 was met at stock's depth and the driver had to
# brake). Latched while any lead is closing: the planner's target passes through with no cap and no jerk limit.
STOCK_FEEL_EMERGENCY = True
STOCK_FEEL_EMERGENCY_A_LEAD = -2.0  # m/s^2
STOCK_FEEL_EMERGENCY_NEED = 2.0  # m/s^2

# ---- Stop ease (owner 2026-10-07, 2f2: every hard stop reached standstill still braking -1.6..-1.7, the clunk) ------
STOP_EASE = True
STOP_EASE_BP = [0.0, 1.0, 3.0]  # m/s
STOP_EASE_V = [-0.5, -1.1, -2.5]  # m/s^2
STOP_EASE_JERK = 2.0  # m/s^3
STOP_EASE_MIN_GAP = 2.0  # m


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
  if not STOCK_FEEL_EMERGENCY or ttc == float('inf'):
    return False
  if active:
    return True
  for lead in leads:
    if _present(lead) and -float(lead.vRel) > STOCK_FEEL_MIN_CLOSING and float(lead.aLeadK) <= STOCK_FEEL_EMERGENCY_A_LEAD:
      return True
  need = lead_stop_need(leads, v_ego)
  depth = float(np.interp(ttc, STOCK_FEEL_DEPTH_BP, STOCK_FEEL_DEPTH_V))
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


def stock_feel_target(leads, prev: float, target: float, dt: float, v_ego: float = 0.0,
                      emergency: bool = False, law_recent: bool = False) -> float:
  """D-086 stock Honda ACC brake law (see the constants above)."""
  if emergency:
    return float(target)
  ttc = brake_onset_ttc(leads, STOCK_FEEL_MIN_CLOSING)
  if ttc == float('inf') and law_recent and STOCK_FEEL_SOFT_EXIT and not any(
      _present(lead) and float(lead.aLeadK) <= STOCK_FEEL_LEAD_STOP_A for lead in leads):
    return brake_onset_limited_target(prev, target, dt, STOCK_FEEL_EXIT_JERK)
  if not ttc > STOCK_FEEL_TTC_FLOOR_S or ttc == float('inf'):
    return brake_onset_limited_target(prev, target, dt, STOCK_FEEL_JERK_OUTSIDE)
  depth = float(np.interp(ttc, STOCK_FEEL_DEPTH_BP, STOCK_FEEL_DEPTH_V))
  if STOCK_FEEL_LEAD_STOP:
    need = lead_stop_need(leads, v_ego, STOCK_FEEL_LEAD_STOP_MAX_T)
    if need > -depth:
      return brake_onset_limited_target(prev, max(target, stock_feel_held_cap(prev, -min(need, 3.5), dt)), dt,
                                        STOCK_FEEL_JERK_OUTSIDE)
  target = max(target, stock_feel_held_cap(prev, depth, dt))
  return brake_onset_limited_target(prev, target, dt, float(np.interp(ttc, STOCK_FEEL_JERK_BP, STOCK_FEEL_JERK_V)))


def stop_ease_distance(v: float) -> float:
  """Distance (m) to stop from v under the STOP_EASE_V floor."""
  n = 20
  dv = max(float(v), 0.0) / n
  return float(sum((i + 0.5) * dv * dv / -float(np.interp((i + 0.5) * dv, STOP_EASE_BP, STOP_EASE_V)) for i in range(n)))


def stop_ease_floor(v_ego: float, d_lead: float | None) -> float | None:
  """Floor (m/s^2) at this speed, or None when off, above STOP_EASE_BP[-1], or when the eased stop would end closer
  than STOP_EASE_MIN_GAP to the lead."""
  if not STOP_EASE or not float(v_ego) < STOP_EASE_BP[-1]:
    return None
  if d_lead is not None and stop_ease_distance(v_ego) > float(d_lead) - STOP_EASE_MIN_GAP:
    return None
  return float(np.interp(max(float(v_ego), 0.0), STOP_EASE_BP, STOP_EASE_V))


def stop_eased_target(prev: float, target: float, floor: float | None, dt: float) -> float:
  """Raise a target below the floor toward it, at most STOP_EASE_JERK * dt per step above prev."""
  if floor is None or not target < floor:
    return float(target)
  return float(min(floor, max(target, prev + STOP_EASE_JERK * dt)))


class HondaBoschALongEnhancement:
  def __init__(self, CP, dt: float):
    self.enabled = active_for(CP)
    self.dt = dt
    self.reset()

  def reset(self):
    self.emergency = False
    self.law_t = 0.0

  def step(self, *, lead_one, lead_two, prev: float, target: float, v_ego: float,
           stopping: bool, standstill: bool, reset_state: bool) -> float:
    """Shape the final acceleration target. `stopping` covers should-stop, force stop and red-light stops."""
    if not self.enabled or reset_state or standstill:
      self.reset()
      return float(target)
    leads = (lead_one, lead_two)
    if stopping:
      self.reset()
    else:
      self.emergency = stock_feel_emergency(leads, v_ego, self.emergency)
      target = stock_feel_target(leads, prev, target, self.dt, v_ego, self.emergency, self.law_t > 0.0)
      self.law_t = STOCK_FEEL_EXIT_HOLD_S if stock_feel_law_active(leads, self.emergency) else max(0.0, self.law_t - self.dt)
    d_lead = float(lead_one.dRel) if _present(lead_one) else None
    return stop_eased_target(prev, target, stop_ease_floor(v_ego, d_lead), self.dt)
