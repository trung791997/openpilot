import os

import pytest

from openpilot.starpilot.system.android_auto import placement
from openpilot.starpilot.system.android_auto.placement import FALLBACK_NICE, PLACEMENT_CHECK_INTERVAL, RENDER_CORE, RendererPlacement

SCHED_IDLE = 5


class FakeKernel:
  """Per-thread affinity and policy, standing in for Linux (and for macOS, which has neither)."""

  def __init__(self, monkeypatch, threads=(100,), mask=(0, 1, 2, 3, 4, 5), idle_allowed=True):
    self.masks = {tid: set(mask) for tid in threads}
    self.main = threads[0]
    self.online = {RENDER_CORE: True}
    self.policy = None
    self.niced = 0
    self.idle_allowed = idle_allowed
    monkeypatch.setattr(os, "SCHED_IDLE", SCHED_IDLE, raising=False)
    monkeypatch.setattr(os, "sched_param", lambda priority: priority, raising=False)
    monkeypatch.setattr(os, "sched_getaffinity", self.getaffinity, raising=False)
    monkeypatch.setattr(os, "sched_setaffinity", self.setaffinity, raising=False)
    monkeypatch.setattr(os, "sched_setscheduler", self.setscheduler, raising=False)
    monkeypatch.setattr(os, "nice", self.nice)

  def getaffinity(self, tid):
    if (tid or self.main) not in self.masks:
      raise OSError(3, "No such process")
    return set(self.masks[tid or self.main])

  def setaffinity(self, tid, mask):
    if not any(self.online.get(core, True) for core in mask):
      raise OSError(22, "Invalid argument")
    self.masks[tid or self.main] = set(mask)

  def setscheduler(self, tid, policy, param):
    if not self.idle_allowed:
      raise OSError(1, "Operation not permitted")
    self.policy = policy

  def nice(self, increment):
    self.niced += increment
    return self.niced

  def make(self, events, **kwargs):
    return RendererPlacement(report=events.append, thread_ids=lambda: list(self.masks),
                             core_online=lambda core: self.online.get(core, True), **kwargs)


@pytest.fixture
def events():
  return []


def test_start_uses_sched_idle_and_pins_every_thread_to_core_6(monkeypatch, events):
  kernel = FakeKernel(monkeypatch, threads=(100, 101, 102))
  kernel.make(events).start()

  assert kernel.policy == SCHED_IDLE and kernel.niced == 0
  assert all(mask == {6} for mask in kernel.masks.values())
  assert events[0] == {"event": "cpu_placement", "policy": "idle", "inherited_cores": [0, 1, 2, 3, 4, 5], "core": 6}
  assert events[1] == {"event": "cpu_placement", "pinned": True, "cores": [6]}


def test_falls_back_to_nice_without_sched_idle(monkeypatch, events):
  kernel = FakeKernel(monkeypatch, idle_allowed=False)
  kernel.make(events).start()

  assert kernel.policy is None and kernel.niced == FALLBACK_NICE
  assert events[0]["policy"] == f"nice{FALLBACK_NICE}"
  assert kernel.masks[100] == {6}


def test_offroad_power_save_then_repin_when_core_6_returns(monkeypatch, events):
  kernel = FakeKernel(monkeypatch, threads=(100, 101))
  kernel.online[RENDER_CORE] = False  # started offroad: big cluster offline
  p = kernel.make(events)
  p.start()
  assert kernel.masks[100] == {0, 1, 2, 3, 4, 5} and len(events) == 1  # left alone, no error

  kernel.online[RENDER_CORE] = True  # onroad: big cluster back
  p.maintain(PLACEMENT_CHECK_INTERVAL)
  assert all(mask == {6} for mask in kernel.masks.values())
  assert events[-1] == {"event": "cpu_placement", "pinned": True, "cores": [6]}

  kernel.online[RENDER_CORE] = False  # offroad again: kernel widens our mask
  kernel.masks = {tid: set(range(8)) for tid in kernel.masks}
  p.maintain(2 * PLACEMENT_CHECK_INTERVAL)
  assert events[-1] == {"event": "cpu_placement", "pinned": False, "reason": "core 6 offline"}

  kernel.online[RENDER_CORE] = True
  p.maintain(3 * PLACEMENT_CHECK_INTERVAL)
  assert all(mask == {6} for mask in kernel.masks.values()) and events[-1]["pinned"]


def test_threads_created_later_are_pinned_and_checks_are_throttled(monkeypatch, events):
  kernel = FakeKernel(monkeypatch)
  p = kernel.make(events)
  p.start()
  count = len(events)

  kernel.masks[200] = {0, 1, 2, 3, 4, 5}  # e.g. a graphics-driver thread started after the pin
  p.maintain(PLACEMENT_CHECK_INTERVAL / 2)
  assert kernel.masks[200] != {6}  # not due yet

  p.maintain(PLACEMENT_CHECK_INTERVAL)
  assert kernel.masks[200] == {6} and len(events) == count + 1

  p.maintain(2 * PLACEMENT_CHECK_INTERVAL)  # nothing changed: nothing logged
  assert len(events) == count + 1


def test_exited_thread_is_ignored(monkeypatch, events):
  kernel = FakeKernel(monkeypatch, threads=(100, 101))
  p = kernel.make(events)
  p.thread_ids = lambda: [100, 101, 999]  # 999 exited between listing and setting
  p.start()
  assert kernel.masks[100] == kernel.masks[101] == {6}


def test_defaults_target_core_6():
  assert placement.RENDER_CORE == 6


def test_normal_scheduling_pins_without_sched_idle(monkeypatch, events):
  # android_autod: core 6 next to the renderer, but never below other work
  kernel = FakeKernel(monkeypatch, threads=(100, 101))
  kernel.make(events, idle=False).start()
  assert kernel.policy is None and kernel.niced == 0
  assert all(mask == {6} for mask in kernel.masks.values())
  assert events[0]["policy"] == "normal"


def test_renderer_moves_to_core_7_only_after_the_jetson_has_driven_for_a_while(monkeypatch, events):
  kernel = FakeKernel(monkeypatch, threads=(100, 101))
  drives = [False]
  p = kernel.make(events, jetson_drives=lambda: drives[0])
  p.start()
  t0 = 1000.0  # time.monotonic() in the renderer
  p.maintain(t0)
  assert all(mask == {6} for mask in kernel.masks.values())

  drives[0] = True
  p.maintain(t0 + 1)
  p.maintain(t0 + 1 + placement.JETSON_SETTLE - 1)
  assert all(mask == {6} for mask in kernel.masks.values())  # not settled yet

  p.maintain(t0 + 1 + placement.JETSON_SETTLE)
  assert all(mask == {7} for mask in kernel.masks.values())
  assert {"event": "cpu_placement", "core": 7, "reason": "jetson drives"} in events
  assert events[-1] == {"event": "cpu_placement", "pinned": True, "cores": [7]}

  drives[0] = False  # modeld runs its own model: back to core 6 at the first check
  p.maintain(t0 + 2 + placement.JETSON_SETTLE)
  assert all(mask == {6} for mask in kernel.masks.values())
  assert events[-1] == {"event": "cpu_placement", "pinned": True, "cores": [6]}

  drives[0] = True  # the settle restarts
  p.maintain(t0 + 3 + placement.JETSON_SETTLE)
  assert all(mask == {6} for mask in kernel.masks.values())


def test_start_never_moves_on_its_own_clock(monkeypatch, events):
  kernel = FakeKernel(monkeypatch)
  p = kernel.make(events, jetson_drives=lambda: True)
  p.start()
  p.maintain(5000.0)  # first real check: the settle starts here, not at start()'s 0
  assert kernel.masks[100] == {6}


def test_android_autod_never_follows_the_jetson(monkeypatch, events):
  kernel = FakeKernel(monkeypatch)
  p = kernel.make(events, idle=False, jetson_drives=lambda: True)
  p.start()
  for t in range(1, 20):
    p.maintain(float(t))
  assert kernel.masks[100] == {6}


def test_core_7_offline_waits_like_core_6(monkeypatch, events):
  kernel = FakeKernel(monkeypatch)
  p = kernel.make(events, jetson_drives=lambda: True)
  p.start()
  p.maintain(100.0)
  kernel.online[placement.JETSON_CORE] = False
  p.maintain(100.0 + placement.JETSON_SETTLE)
  assert kernel.masks[100] == {6} and events[-1] == {"event": "cpu_placement", "core": 7, "reason": "jetson drives"}
  kernel.online[placement.JETSON_CORE] = True
  p.maintain(101.0 + placement.JETSON_SETTLE)
  assert kernel.masks[100] == {7}


def test_no_jetlink_reads_as_not_driving(monkeypatch):
  import builtins
  real = builtins.__import__

  def no_jetlink(name, *args, **kwargs):
    if name.startswith("jetlink"):
      raise ImportError(name)
    return real(name, *args, **kwargs)
  monkeypatch.setattr(builtins, "__import__", no_jetlink)
  assert placement._jetson_drives() is False
