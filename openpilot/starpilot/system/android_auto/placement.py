"""Runs the car renderer on core 6 at the lowest priority.
Safeguards:
- SCHED_IDLE, so camerad and the comma UI on core 6 always preempt it.
- Core 6 again since 2026-10-14. On 2026-10-08 the comma UI took 65-75 % of it
  and the renderer moved to core 7, but there it waited for a core 47 % of the
  time behind modeld and dmonitoringmodeld (core 7 at 78 %). With the UI now
  at 8-14 %, core 6 runs 16-24 % busy, and core 7 is left to the models and
  Jetlink's frame loop.
- Core 7 while the Jetson drives (2026-10-25). With the driving model on the
  Jetson, core 7 ran 24 % busy while core 6 ran 95 % (camerad, controlsd, the
  comma UI, android_autod), and the renderer, SCHED_IDLE, reached the car at
  10-17 fps while driving against 19.6 parked (android_auto session logs
  2026-10-24, encode peak under 31 ms). It moves only after the Jetson has
  driven for JETSON_SETTLE seconds, and back to core 6 at the first check it
  does not: if modeld runs its own model, core 7 is the models' again.
- Every renderer thread is placed, including ones the graphics driver starts later.
- Parked, power saving turns cores 4-7 off. The pin is re-applied every
  PLACEMENT_CHECK_INTERVAL seconds once core 6 is back, like selfdrive/ui/ui.py.
"""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

RENDER_CORE = 6
JETSON_CORE = 7
JETSON_SETTLE = 5.0  # s the Jetson must have driven before the renderer moves to JETSON_CORE
PLACEMENT_CHECK_INTERVAL = 1.0  # s
FALLBACK_NICE = 10  # if SCHED_IDLE is unavailable


def _thread_ids() -> list[int]:
  try:
    return [int(tid) for tid in os.listdir("/proc/self/task")]
  except OSError:
    return [0]


def _core_online(core: int) -> bool:
  if core == 0:
    return True
  try:
    with open(f"/sys/devices/system/cpu/cpu{core}/online") as f:
      return f.read().strip() == "1"
  except OSError:
    return False


def _jetson_drives() -> bool:
  """Whether the Jetson's model drives modeld's frames now (jetlink's runtime record); False on any doubt."""
  try:
    from jetlink.openpilot.status import runtime
    return runtime().active
  except Exception:
    return False


class RendererPlacement:
  """Keeps every thread of this process on RENDER_CORE while that core is online:
  SCHED_IDLE for the renderer, or normal scheduling (idle=False) for
  android_autod, whose stream thread otherwise sticks on whichever isolated
  core it first woke on (core 7, waiting ~32 % of the time behind the models).
  Only the SCHED_IDLE renderer follows the Jetson to JETSON_CORE; android_autod
  stays on RENDER_CORE, where it cannot delay a model modeld falls back to."""

  def __init__(self, core: int = RENDER_CORE, report: Callable[[dict[str, Any]], None] | None = None,
               thread_ids: Callable[[], list[int]] = _thread_ids, core_online: Callable[[int], bool] = _core_online,
               idle: bool = True, jetson_drives: Callable[[], bool] = _jetson_drives):
    self.home_core = core
    self.core = core
    self.idle = idle
    self.report = report or (lambda _event: None)
    self.thread_ids = thread_ids
    self.core_online = core_online
    self.next_check = 0.0
    self.pinned = False
    self.jetson_drives = jetson_drives
    self.jetson_since: float | None = None

  def start(self) -> None:
    """Call first, before any thread exists, so later threads inherit the policy."""
    if not hasattr(os, "sched_setaffinity"):
      self.report({"event": "cpu_placement", "pinned": False, "reason": "no CPU affinity on this platform"})
      self.next_check = float("inf")
      return
    inherited = sorted(os.sched_getaffinity(0))
    policy = "idle" if self.idle else "normal"
    try:
      if self.idle:
        os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
    except (AttributeError, OSError):
      policy = f"nice{FALLBACK_NICE}"
      try:
        os.nice(FALLBACK_NICE)
      except OSError:
        policy = "default"
    self.report({"event": "cpu_placement", "policy": policy, "inherited_cores": inherited, "core": self.core})
    self.maintain(0.0, choose_core=False)  # on the caller's clock from the first maintain()

  def maintain(self, now: float, choose_core: bool = True) -> None:
    if now < self.next_check:
      return
    self.next_check = now + PLACEMENT_CHECK_INTERVAL
    if choose_core:
      self._choose_core(now)
    if not self.core_online(self.core):
      if self.pinned:
        self.report({"event": "cpu_placement", "pinned": False, "reason": f"core {self.core} offline"})
      self.pinned = False
      return

    target = {self.core}
    changed = False
    for tid in self.thread_ids():
      try:
        if os.sched_getaffinity(tid) != target:
          os.sched_setaffinity(tid, target)
          changed = True
      except OSError:
        pass  # the thread exited
    if changed or not self.pinned:
      self.report({"event": "cpu_placement", "pinned": True, "cores": sorted(os.sched_getaffinity(0))})
    self.pinned = True

  def _choose_core(self, now: float) -> None:
    if not self.idle or self.home_core != RENDER_CORE:
      return
    if self.jetson_drives():
      self.jetson_since = now if self.jetson_since is None else self.jetson_since
    else:
      self.jetson_since = None
    core = JETSON_CORE if self.jetson_since is not None and now - self.jetson_since >= JETSON_SETTLE else self.home_core
    if core != self.core:
      self.core = core
      self.pinned = False
      self.report({"event": "cpu_placement", "core": core, "reason": "jetson drives" if core == JETSON_CORE else "jetson not driving"})
