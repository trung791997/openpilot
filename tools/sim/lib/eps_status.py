"""The EPS's STEER_STATUS for the MetaDrive lateral sim (SIM_EPS_STATUS), so the car's own no-torque fault flicker can be
driven into the loop (sim expansion plan, driver-override capability a).

STEER_STATUS 2 = NO_TORQUE_ALERT_1 sets carState.steerFaultTemporary on both opendbc Honda carstate paths (the plain Bosch
one lists it as temporary; BOSCH_ALT_RADAR treats anything but NORMAL and an expected low-speed lockout as temporary):
selfdrived drops latActive, the ClarityEps core resets and the limiter re-seeds when it comes back.
Until now the sim sent STEER_STATUS 0 in every frame, so none of that was ever exercised.

Road fit (limited road evidence, routes 290-297, carState at 100 Hz, Kevin 2026-09-29): the fault is a driver-torque
threshold. The first fault frame is never below |STEER_TORQUE_SENSOR| 3152 (p10 3203, median 3314); P(fault frame | |tq|)
is 0 below 3000, 0.03 at 3000-3200, 0.52 at 3200-3400, 0.91 at 3400-4000. Mostly below 4.5 m/s (that is where drivers
reach 3200 against the assist), so there is no speed gate by default. Of 92 OFF gaps between fault runs, 65 have the
torque dip below 3150 (the hysteresis below), and 27 keep it above 3150 throughout: the EPS itself clears the alert for
0.04-0.17 s (median 0.06) and raises it again. Of the 98 ON runs, 70 end with |tq| still >= 3150 (the EPS ended them;
median 0.09 s, max 5.53 s) and 28 end on a torque dip, which the hysteresis already produces. Those 28 are right-censored
EPS lengths (the EPS would have ended them later), so dropping them leans the drawn ON lengths slightly short, most in the
tail (James review; disclosed in the 2b prereg, which gates the p50 only). The threshold mode resamples
each ON run's cap from the 70 EPS-ended lengths and each EPS-side gap from the 27 high-torque gaps (road frame counts
below, drawn directly: no interpolation between deciles, which put 10 % of draws at a uniform 0.72-6.96 s; James/John
review 2026-09-29).
Seed: "seed" (default 0). Twins with the same seed and torque trace flicker the same way; in a multi-seed row the runner
sets seed from the tag (PA1 -> 1, ...) or the prereg says the draws are common across seeds.
Clock: one update = DT 0.01 s, the same fixed step as driver_model; the bridge loop runs at a measured 96-99 Hz, so wall
durations are 1-4 % longer than the drawn ones.
Soft disable (James, car_specific.py): steerFaultTemporary with steeringPressed false for >= 1.5 s raises the
steerTempUnavailable soft disable. Threshold mode cannot get there (ON needs |tq| > 3150, which is pressed); a scripted
template with a long ON (294s14 1.80 s) can if the hand lets go mid-run. The scorer needs a rule for a mid-episode
disengage before numbers. While disengaged the status is forced to 0 and this model does not step, so the rest of a
template does not play, and its state is not reset on re-engage (single-engage episodes only).

Modes (SIM_EPS_STATUS is JSON; unset = today's behaviour, STEER_STATUS 0 always):
  {"threshold": true}                       status 2 once |driver tq| > tq_on (3200); back to 0 when it falls below
                                            tq_off (3150) or the drawn ON length ends (then 0 for the drawn EPS gap)
                                            "seed" (default 0) fixes the draws
  {"events": [{"after_press_s": 1.0, "on": [0.09, 0.09, 0.30], "off": [0.09, 0.06]}]}
                                            scripted: ON/OFF durations alternate from after_press_s past the first
                                            press of the episode (not the nearest press; one event per run)
                                            ("at_s" = seconds after the first engaged step instead).
                                            "template": "290s11" / "294s14" / "297s56a" / "297s56b" picks a road pattern.
  "v_max": 4.5                              only below this speed (off by default)
  "cut_assist": true                        while status is 2 the plant gets no openpilot torque, from the frame the
                                            status rises (off: it gets it until carOutput follows latActive, a frame
                                            or two). No road measure of whether the C020 EPS drops our command in the
                                            fault, so off by default, one A/B in the 2b prereg, every run names it.
Both modes may be given; the status is 2 when either says so. Sim evidence only.
"""
import json

import numpy as np

DT = 0.01
NORMAL, NO_TORQUE_ALERT_1 = 0, 2

# Road flicker patterns (carState 100 Hz, route-relative seconds within the segment). after_press_s = time since |tq| last
# rose above 300 before the first fault frame; on/off = the fault runs and the gaps between them. One episode each.
TEMPLATES = {
  "290s11": {"after_press_s": 1.04, "on": [0.08, 0.09], "off": [0.09]},                  # 52.19 s, v 1.6 m/s
  "294s14": {"after_press_s": 2.06, "on": [0.21, 0.01, 1.80], "off": [0.06, 0.27]},      # 10.31 s, v 2.4 m/s
  "297s56a": {"after_press_s": 1.31, "on": [0.02, 0.01, 0.06, 0.02, 0.02, 0.07, 0.13, 0.04],
              "off": [0.06, 0.09, 0.16, 0.09, 0.08, 0.18, 0.32]},                         # 21.51 s, v 7.7 -> 6.4 m/s
  "297s56b": {"after_press_s": 0.73, "on": [0.04, 0.67], "off": [0.06]},                 # 32.46 s, v 3.5 m/s
}
# Road frame counts (100 Hz, routes 290-297): fault runs that ended with |tq| still >= 3150 (n 70), and gaps between
# fault runs with |tq| > 3150 throughout (n 27).
ON_FRAMES = [1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 6, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10,
             10, 11, 13, 13, 13, 13, 14, 16, 17, 17, 21, 21, 22, 23, 25, 32, 33, 36, 40, 43, 46, 51, 53, 58, 66, 85, 89, 95, 118,
             164, 179, 553]
GAP_FRAMES = [4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 8, 8, 9, 9, 12, 15, 17]


class EpsStatus:
  def __init__(self, spec: str | dict):
    s = json.loads(spec) if isinstance(spec, str) else spec
    self.threshold = bool(s.get("threshold", False))
    self.tq_on = float(s.get("tq_on", 3200.0))
    self.tq_off = float(s.get("tq_off", 3150.0))
    self.rng = np.random.default_rng(int(s.get("seed", 0)))
    self.v_max = s.get("v_max")
    self.cut_assist = bool(s.get("cut_assist", False))
    self.events = [dict(TEMPLATES[e["template"]], **{k: v for k, v in e.items() if k != "template"}) if "template" in e else dict(e)
                   for e in s.get("events", [])]
    self.n = 0  # steps taken; t = n * DT, not a running float sum
    self.t = 0.0
    self.press_t0 = None
    self.thr_on = False
    self.thr_left = 0  # frames left in the current ON run (thr_on) or EPS-side gap (not thr_on); integer, so twins stay
                       # bit-identical (James review)
    self.seq = None  # [(t_on, t_off), ...] of the scripted event in progress
    self.status = NORMAL
    self.log = []  # (t_on, t_off) of each status-2 run, for scoring

  def _scripted(self) -> bool:
    if self.seq is None and self.events:
      e = self.events[0]
      if "at_s" in e:
        t0 = e["at_s"]
      elif self.press_t0 is not None:
        t0 = self.press_t0 + e["after_press_s"]
      else:
        return False
      n0 = round(t0 / DT)
      if self.n >= n0:
        self.events.pop(0)
        edges, n = [], n0  # step numbers, rounded once per duration, so a 0.01 s run is exactly one step
        for k, on in enumerate(e["on"]):
          edges.append((n, n + round(on / DT)))
          n = edges[-1][1] + (round(e["off"][k] / DT) if k < len(e["off"]) else 0)
        self.seq = edges
    if self.seq is None:
      return False
    if self.n >= self.seq[-1][1]:
      self.seq = None
      return False
    return any(a <= self.n < b for a, b in self.seq)

  def draw(self, frames) -> int:
    return int(frames[int(self.rng.integers(len(frames)))])

  def update(self, driver_tq: float, v_ego: float, pressed_started: bool = False) -> int:
    """One 100 Hz step. pressed_started: the driver model has started its first press (for after_press_s)."""
    self.n += 1
    self.t = self.n * DT
    if pressed_started and self.press_t0 is None:
      self.press_t0 = self.t
    on = False
    if self.threshold:
      if self.thr_on:
        self.thr_left -= 1
        if abs(driver_tq) < self.tq_off:
          self.thr_on, self.thr_left = False, 0
        elif self.thr_left <= 0:
          self.thr_on, self.thr_left = False, self.draw(GAP_FRAMES)
      else:
        self.thr_left -= 1
        if abs(driver_tq) > self.tq_on and self.thr_left <= 0:
          self.thr_on, self.thr_left = True, self.draw(ON_FRAMES)
      on = self.thr_on
    on = self._scripted() or on
    if self.v_max is not None and v_ego >= self.v_max:
      on = False
    new = NO_TORQUE_ALERT_1 if on else NORMAL
    if new != self.status:
      if new == NO_TORQUE_ALERT_1:
        self.log.append((self.t, None))
      else:
        self.log[-1] = (self.log[-1][0], self.t)
    self.status = new
    return self.status
