"""The last turn, kept on the car's screens through a short GPS gap.

navigationd says 'waitingForLocation' the moment it has no usable fix, and then publishes no step. On the head unit's
car GPS that happens for a few seconds at a time: drive 00000075--4ad120a8a2 (2026-10-31) lost the step for 3.8 s
when the car's fixes stopped for ~7 s at 26 mph, and for 0.2 s on one unusable fix at 46 mph. Each time the dash
dropped its turn card and route, then brought them back. The displays hold the last step instead, with the
distance counted down at the car's speed, for up to HOLD_NS.

Display only. navigationd's message is unchanged, so nothing that steers, brakes or picks a speed limit sees a
held step. Only a gap holds: arrival, End navigation, a new destination and rerouting clear the step as before.
"""
from __future__ import annotations

HOLD_NS = 20_000_000_000      # longest gap held; past this the step is dropped as navigationd says
GAP_STATUSES = frozenset({'waitingForLocation'})  # route still set, no fix: the only status a gap produces
EMIT_NS = 1_000_000_000       # a held step's distances move once a second, like navigationd's own steps
MAX_STEP_NS = 1_000_000_000   # a longer pause between calls adds no distance (a stalled reader is not a drive)


class GuidanceHold:
  """Feed it every read: hold(status, key, step, now_ns, speed, at_ns) -> (step to show or None, held).

  status: navigationd's status. key: what identifies the route (session, revision); a gap holds only the same
  route. step: the consumer's own dict while guiding (None otherwise), with 'distance', 'remaining' (m) and
  'duration' (s) where it has them. speed: the car's speed (m/s) for the countdown. at_ns: when navigationd
  published the step (its distances are as of then); the countdown and the hold run from it. Default now_ns.
  """

  def __init__(self, hold_ns: int = HOLD_NS):
    self.hold_ns = hold_ns
    self.key = None
    self.step: dict | None = None
    self.since = self.last = self.emitted_at = 0
    self.travelled = 0.
    self.shown: dict | None = None

  def clear(self) -> None:
    self.key, self.step, self.shown = None, None, None

  def __call__(self, status, key, step: dict | None, now_ns: int, speed: float = 0.,
               at_ns: int | None = None) -> tuple[dict | None, bool]:
    if status == 'guiding' and step is not None:
      self.key, self.step, self.shown = key, step, None
      self.since = self.last = min(now_ns, at_ns) if at_ns is not None and at_ns > 0 else now_ns
      self.travelled = 0.
      return step, False
    if status not in GAP_STATUSES or self.step is None or key != self.key or now_ns - self.since > self.hold_ns:
      self.clear()
      return None, False
    if now_ns > self.last:
      self.travelled += max(0., float(speed)) * min(now_ns - self.last, MAX_STEP_NS) / 1e9
    self.last = max(self.last, now_ns)
    if self.shown is None or now_ns - self.emitted_at >= EMIT_NS:
      self.shown, self.emitted_at = self._counted_down(now_ns), now_ns
    return self.shown, True

  def _counted_down(self, now_ns: int) -> dict:
    step = dict(self.step)
    for name in ('distance', 'remaining'):
      if isinstance(step.get(name), (int, float)):
        value = max(0., step[name] - self.travelled)
        step[name] = round(value, 1) if isinstance(step[name], float) else round(value)
    if isinstance(step.get('duration'), (int, float)):
      value = max(0., step['duration'] - (now_ns - self.since) / 1e9)
      step['duration'] = round(value, 1) if isinstance(step['duration'], float) else round(value)
    return step
