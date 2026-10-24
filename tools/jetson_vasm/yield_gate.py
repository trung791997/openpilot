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


class YieldGate:
  def __init__(self, path: str = GPU_LOAD, load_max: float = LOAD_MAX, hold_s: float = HOLD_S,
               clock=time.monotonic, start: bool = True):
    self.path, self.load_max, self.hold_s, self.clock = path, load_max, hold_s, clock
    self.load = 0.0
    self.until = 0.0
    self.reason = ''
    self.stats = {'load_yields': 0, 'slow_yields': 0}
    self._passes: dict[int, collections.deque] = {}
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
    if not self.yielding():
      self.stats[f'{why}_yields'] += 1
    self.reason = reason
    self.until = self.clock() + self.hold_s

  def _sample_loop(self):
    while True:
      self.sample(self.read_load())
      time.sleep(SAMPLE_S)
