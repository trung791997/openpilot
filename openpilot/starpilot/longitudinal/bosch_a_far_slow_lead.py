"""Bosch-A far slow lead: start slowing when the gap to a far car is closing faster than its speed reading allows.

Owner 2026-11-03, route 84 (00000084--e551616ebe), both bookmarks. At 22-25 m/s a slow car was in view from 110-130 m,
but its published speed read 14-20 m/s until the radar locked at 82-85 m and read 2.8-4.9. The planner then stepped
from about -0.5 to -3.3/-3.5 in one tick (car peak -4.2/-4.4); coast first correctly bypassed (TTC under 5 s).
  4:52 (seg 4): dRel 120.2 -> 92.0 m over 4:49.1-4:50.5 at vEgo 22.5 (the gap closed at ~20 m/s: a car at ~2 m/s)
    while vLead read 14-16. The car then sped up to ~12 m/s ("a sudden brake when the car is nowhere close").
  12:37 (seg 12): a lead slowing 20 -> 9 m/s; vLead read 18-20 down to 95 m, the model agrees at ~10 after the lock.
Starting the slowdown early, capped gentle, was the fix the owner approved (2026-11-03: "go ahead and build it").

Channel (D-048 amendment standing rule): the closing speed here is the least-squares slope of the published lead dRel
over WINDOW_S, i.e. the range channel, not vLead/U11. It is not affected by the U11 rail: a railed vLead only makes the
mismatch larger, and the brake is bounded at MAX_EARLY_ACCEL whatever the mismatch. When dRel itself is wrong (a track
on the wrong scatterer, D-048's 231 case, or a lead identity switch) the worst this does is MAX_EARLY_ACCEL for
HOLD_S past the last agreeing window; a per-tick dRel jump over MAX_STEP_M restarts the window, and a noisy fit
(residual RMS over MAX_RESIDUAL_M) never triggers. Radar-side, D-053 (RangeDerivedVrel, default off) corrects U11 the
same direction inside radard; this is the planner-side bound and composes with it.

It only lowers the target (never adds gas) and never touches dRel, vLead or the radar points (D-041/D-042).
Static evidence only; not replayed or driven yet.
"""
from collections import deque

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.longitudinal.bosch_a_toggles import vehicle_supported

MIN_V_EGO = 15.0         # m/s; the far-lead case is at road speed (84: 22.5 and 24.7)
MIN_D_M = 60.0           # inside this the radar lead reads its speed; the planner owns it (84 locks at 82-85 m)
MAX_D_M = 160.0
WINDOW_S = 1.0           # LSQ window on dRel; 84 4:49.1-4:50.1 closes 20 m/s over it
MAX_STEP_M = 3.0         # per-tick dRel jump = a different object (84 12:34.56: 103.5 -> 112.4 m); restart
MAX_RESIDUAL_M = 3.0     # fit residual RMS; noisier than this is not a measured closing speed
MIN_CLOSING = 5.0        # m/s
MIN_MISMATCH = 3.0       # m/s; vLead faster than the gap allows by at least this (84 12:33: ~3.3)
STOP_DISTANCE_M = 6.0    # long_mpc STOP_DISTANCE
MIN_ROOM_M = 5.0
MAX_EARLY_ACCEL = -1.0   # m/s^2; agreed cap: a gentle early slowdown, the planner brakes harder once the lead reads
HOLD_S = 1.0             # keep the last bound this long after the window stops agreeing (radar/vision flicker)


def closest_lead(leads):
  present = [lead for lead in leads if lead is not None and bool(getattr(lead, 'present', False)) and
             float(lead.dRel) > 0.0]
  return min(present, key=lambda lead: float(lead.dRel)) if present else None


def lsq_slope(samples):
  """samples: (t, d). Returns (slope, residual RMS)."""
  n = len(samples)
  mt = sum(t for t, _ in samples) / n
  md = sum(d for _, d in samples) / n
  stt = sum((t - mt) ** 2 for t, _ in samples)
  if stt <= 0.0:
    return 0.0, float('inf')
  slope = sum((t - mt) * (d - md) for t, d in samples) / stt
  rms = (sum((d - (md + slope * (t - mt))) ** 2 for t, d in samples) / n) ** 0.5
  return slope, rms


class BoschAFarSlowLead:
  def __init__(self, CP, dt: float = DT_MDL):
    self.enabled = vehicle_supported(CP) and not bool(getattr(CP, 'radarUnavailable', True))
    self.dt = dt
    self.window = deque(maxlen=max(2, int(round(WINDOW_S / dt)) + 1))
    self.t = 0.0
    self.bound = None      # last early-slowdown target; None when not active (replay/debug)
    self.hold_left = 0.0
    self.v_est = None      # lead speed implied by the gap (replay/debug)

  def reset(self):
    self.window.clear()
    self.bound = None
    self.hold_left = 0.0
    self.v_est = None

  @property
  def active(self) -> bool:
    return self.bound is not None

  def _measure(self, lead, v_ego: float, follow_s: float):
    d_rel = float(lead.dRel)
    if self.window and abs(d_rel - self.window[-1][1]) > MAX_STEP_M:
      self.window.clear()
    self.window.append((self.t, d_rel, v_ego))
    if len(self.window) < self.window.maxlen:
      return None
    slope, rms = lsq_slope([(t, d) for t, d, _ in self.window])
    closing = -slope
    if rms > MAX_RESIDUAL_M or closing < MIN_CLOSING:
      return None
    v_mean = sum(v for _, _, v in self.window) / len(self.window)
    v_est = max(0.0, v_mean - closing)
    if float(lead.vLead) - v_est < MIN_MISMATCH:
      return None
    self.v_est = v_est
    room = max(d_rel - (follow_s * v_est + STOP_DISTANCE_M), MIN_ROOM_M)
    return max(-(v_ego - v_est) ** 2 / (2.0 * room), MAX_EARLY_ACCEL)

  def step(self, *, lead_one, lead_two, target: float, v_ego: float, follow_s: float, reset_state: bool) -> float:
    target = float(target)
    self.t += self.dt
    if not self.enabled or reset_state or v_ego < MIN_V_EGO:
      self.reset()
      return target
    lead = closest_lead((lead_one, lead_two))
    if lead is None or not (MIN_D_M <= float(lead.dRel) <= MAX_D_M):
      self.reset()
      return target
    bound = self._measure(lead, v_ego, follow_s)
    if bound is not None:
      self.bound, self.hold_left = bound, HOLD_S
    elif self.bound is not None:
      self.hold_left -= self.dt
      if self.hold_left <= 0.0:
        self.bound = None
    return target if self.bound is None else min(target, self.bound)
