"""The Jetson's add-ons (V-ASM, speed-limit vision) give the GPU back to Jetlink's driving model.

The driving model comes first; the add-ons only use headroom it leaves. While the gate yields, a server
answers every request with STATUS_YIELD and sends yielding hellos, and the comma stops sending and runs its
own CPU model (spot_monitor/jetson_link.py, speed_limits/vision/jetson_net.py) until a plain hello.

It yields, for at least HOLD_S, when either:
  - the GPU's busy fraction (sysfs load, 0..1000, averaged over about 1 s) is above LOAD_MAX, or
  - this add-on's own pass took more than SLOW_FACTOR times its quiet baseline (the fastest of its last
    BASELINE_N passes) and at least SLOW_MIN_US more: someone else is on the GPU.
jetlink-server reports no per-frame latency this process can read, so the gate cannot see the driving
model's own deadline, only the contention it causes. CUDA stream priorities do not help here: they rank
streams within one process's context, and jetlink-server is another process.

With a log callback it reports, for tuning LOAD_MAX/HOLD_S on the road, without changing what it decides:
  - each yield as it ends: how long, what started it, and what extended it (each new trigger restarts HOLD_S),
  - every SUMMARY_S: the fraction of that time spent yielding, the GPU load's mean and peak, and the episodes.
"""
from __future__ import annotations

import collections
import threading
import time

GPU_LOAD = '/sys/devices/platform/gpu.0/load'
LOAD_MAX = 0.80
SLOW_FACTOR = 2.0
SLOW_MIN_US = 2000
HOLD_S = 5.0
BASELINE_N = 200
SAMPLE_S = 0.05
EMA_ALPHA = SAMPLE_S / 1.0   # about a 1 s average
SUMMARY_S = 60.0


class YieldGate:
  def __init__(self, path: str = GPU_LOAD, load_max: float = LOAD_MAX, hold_s: float = HOLD_S,
               clock=time.monotonic, start: bool = True, log=None):
    self.path, self.load_max, self.hold_s, self.clock, self.log = path, load_max, hold_s, clock, log
    self.load = 0.0
    self.until = 0.0
    self.reason = ''
    self.stats = {'load_yields': 0, 'slow_yields': 0}
    self._passes: dict[int, collections.deque] = {}
    self._lock = threading.Lock()
    self._episode = None   # [start, first reason, Counter of extending triggers, start of its unsummarised part]
    self._window = self._new_window(self.clock())
    if start:
      threading.Thread(target=self._sample_loop, name='yield-gate', daemon=True).start()

  def read_load(self) -> float | None:
    try:
      with open(self.path) as f:
        return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
      return None

  def sample(self, load: float | None):
    if load is None:
      return
    self.load += EMA_ALPHA * (load - self.load)
    if self.load > self.load_max:
      self._hold('load', f'gpu {self.load:.0%}')

  def note(self, kind: int, us: int):
    """One pass of this add-on took us microseconds."""
    d = self._passes.setdefault(kind, collections.deque(maxlen=BASELINE_N))
    base = min(d) if len(d) >= 10 else None
    d.append(us)
    if base is not None and us > SLOW_FACTOR * base and us - base >= SLOW_MIN_US:
      self._hold('slow', f'pass {us} us vs {base} us quiet')

  def yielding(self) -> bool:
    return self.clock() < self.until

  def _hold(self, why: str, reason: str):
    with self._lock:
      if not self.yielding():
        self.stats[f'{why}_yields'] += 1
        self._end_episode()
        self._episode = [self.clock(), f'{why}: {reason}', collections.Counter(), self.clock()]
      elif self._episode is not None:
        self._episode[2][why] += 1
      self.reason = reason
      self.until = self.clock() + self.hold_s

  @staticmethod
  def _new_window(now: float) -> dict:
    return {'start': now, 'yield_s': 0.0, 'samples': 0, 'load_sum': 0.0, 'load_max': 0.0, 'episodes': []}

  def _end_episode(self):
    """The episode that has run out, if any, logged and counted toward the window (at its true end, until)."""
    if self._episode is None or self.yielding():
      return
    start, first, extended, unsummarised = self._episode
    self._episode = None
    seconds = self.until - start
    self._window['yield_s'] += self.until - unsummarised
    self._window['episodes'].append(round(seconds, 1))
    if self.log is not None:
      more = ', '.join(f'{n} {why}' for why, n in sorted(extended.items())) or 'nothing'
      self.log(f'yielded {seconds:.1f} s: started by {first}; extended by {more}')

  def tick(self, load: float | None = None):
    """Per sample: close a finished episode, account the load, and every SUMMARY_S log the window."""
    with self._lock:
      self._end_episode()
      w = self._window
      if load is not None:
        w['samples'] += 1
        w['load_sum'] += load
        w['load_max'] = max(w['load_max'], load)
      now = self.clock()
      if now - w['start'] < SUMMARY_S:
        return
      yield_s = w['yield_s']
      if self._episode is not None:   # the open episode's share of this window
        yield_s += now - self._episode[3]
        self._episode[3] = now
      if self.log is not None:
        mean = w['load_sum'] / w['samples'] if w['samples'] else float('nan')
        self.log(f"yielding {yield_s / (now - w['start']):.0%} of the last {now - w['start']:.0f} s; gpu load mean "
                 f"{mean:.0%} peak {w['load_max']:.0%}; ended yields (s) {w['episodes']}; stats {self.stats}")
      self._window = self._new_window(now)

  def _sample_loop(self):
    while True:
      load = self.read_load()
      self.sample(load)
      self.tick(load)
      time.sleep(SAMPLE_S)
