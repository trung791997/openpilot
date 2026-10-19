from __future__ import annotations

from functools import lru_cache
import math
from pathlib import Path
import time


DEVICE_BUSY_MAX_CPU_USAGE_PERCENT = 89.0
DEVICE_BUSY_AVG_CPU_USAGE_PERCENT = 74.0
DEVICE_BUSY_HOT_CORE_COUNT = 4

_throttle_state: dict[str, dict[str, float]] = {}


@lru_cache(maxsize=1)
def _online_cpu_count() -> int | None:
  try:
    spec = Path("/sys/devices/system/cpu/online").read_text(encoding="utf-8").strip()
    count = 0
    for group in spec.split(","):
      bounds = group.split("-", maxsplit=1)
      first = int(bounds[0])
      last = int(bounds[-1])
      if last < first:
        return None
      count += last - first + 1
    return count or None
  except (OSError, ValueError):
    return None


def _online_cpu_usage(cpu_usage, cores=None) -> list[float]:
  usage = [float(value) for value in cpu_usage]
  online_count = _online_cpu_count()
  if online_count is not None and online_count < len(usage):
    usage = usage[:online_count]

  if cores is not None:
    usage = [usage[core] for core in cores if 0 <= core < len(usage)]
  return usage


class OwnCpuPercent:
  """This process's CPU use, in percent of one core, low-passed over ~tau s so it spans hardwared's psutil window."""

  def __init__(self, tau=1.0, clock=time.monotonic, cpu=time.process_time):
    self.tau, self._clock, self._cpu = tau, clock, cpu
    self._last = None
    self.percent = 0.0

  def update(self):
    now, cpu = self._clock(), self._cpu()
    if self._last is not None and now > self._last[0]:
      dt = now - self._last[0]
      self.percent += (1.0 - math.exp(-dt / self.tau)) * (100.0 * (cpu - self._last[1]) / dt - self.percent)
    self._last = (now, cpu)
    return self.percent


def device_cpu_throttle_factor(cpu_usage, name="vision", cores=None, own_percent=0.0):
  """Return a process-local, low-pass-filtered CPU throttle factor.
  own_percent: the caller's own CPU (percent of one core, OwnCpuPercent), spread evenly over the cores and taken out,
  so a process confined to its cores backs off for the others' load and not for its own (V-ASM, 2026-10-19: alone on
  core 2 it read its own ~30% and stretched its interval ~1.5x on a 76-80% busy core)."""
  usage = _online_cpu_usage(cpu_usage, cores=cores)
  if not usage:
    return 1.0
  if own_percent > 0.0:
    usage = [max(0.0, u - own_percent / len(usage)) for u in usage]

  now = time.monotonic()
  state = _throttle_state.setdefault(name, {
    "factor": 1.0,
    "last_time": now,
    "last_logged_factor": 1.0,
  })
  dt = max(0.0, now - state["last_time"])
  state["last_time"] = now

  average = sum(usage) / len(usage)
  hot_cores = sum(core >= DEVICE_BUSY_MAX_CPU_USAGE_PERCENT for core in usage)
  target_factor = _compute_throttle_factor(average, hot_cores)

  alpha = min(1.0 - math.exp(-0.8 * dt), 1.0)
  state["factor"] = min(max(target_factor * alpha + state["factor"] * (1.0 - alpha), 1.0), 4.0)

  if state["factor"] >= state["last_logged_factor"] + 0.6:
    print(f"[{name}] CPU throttle factor={state['factor']:.1f}x (avg={average:.0f}%, hot={hot_cores}/{len(usage)})")
    state["last_logged_factor"] = state["factor"]
  elif state["factor"] <= 1.05 and state["last_logged_factor"] > 1.05:
    print(f"[{name}] CPU recovered")
    state["last_logged_factor"] = 1.0

  return state["factor"]


def _compute_throttle_factor(average, hot_cores):
  average_factor = 1.0 if average < DEVICE_BUSY_AVG_CPU_USAGE_PERCENT else 1.0 + (average - DEVICE_BUSY_AVG_CPU_USAGE_PERCENT) / 8.0
  hot_factor = 1.0 + max(0, hot_cores - DEVICE_BUSY_HOT_CORE_COUNT + 1) * 0.5
  return min(max(average_factor, hot_factor), 4.0)
