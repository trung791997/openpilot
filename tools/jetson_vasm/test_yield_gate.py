"""yield_gate.YieldGate on a fake clock, no GPU: python3 -m pytest tools/jetson_vasm/test_yield_gate.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import yield_gate as G  # noqa: E402


class Clock:
  t = 100.0

  def __call__(self):
    return self.t


def _gate():
  clock = Clock()
  return G.YieldGate(path='/nonexistent', clock=clock, start=False), clock


def test_quiet_gpu_never_yields():
  gate, _ = _gate()
  for _ in range(100):
    gate.sample(0.3)
    gate.note(0, 3000)
  assert not gate.yielding()


def test_a_busy_gpu_yields_for_hold_then_returns():
  gate, clock = _gate()
  for _ in range(60):   # about 3 s at 0.9: the 1 s average crosses 0.8
    gate.sample(0.9)
  assert gate.yielding() and gate.stats['load_yields'] == 1
  for _ in range(100):
    gate.sample(0.1)
  clock.t += G.HOLD_S - 0.1
  assert gate.yielding()
  clock.t += 0.2
  assert not gate.yielding()


def test_one_busy_sample_does_not_yield():
  gate, _ = _gate()
  gate.sample(1.0)
  assert not gate.yielding()


def test_a_slow_pass_yields_a_fast_one_does_not():
  gate, _ = _gate()
  for _ in range(20):
    gate.note(0, 3000)
  gate.note(0, 5500)    # < 2x
  assert not gate.yielding()
  gate.note(0, 7000)    # > 2x and > 2 ms more
  assert gate.yielding() and gate.stats['slow_yields'] == 1


def test_kinds_keep_their_own_baseline():
  gate, _ = _gate()
  for _ in range(20):
    gate.note(0, 3000)
    gate.note(1, 600)
  gate.note(1, 2000)    # > 2x its own 600 us but not 2 ms more
  assert not gate.yielding()


def test_no_sysfs_reads_none():
  gate, _ = _gate()
  assert gate.read_load() is None
  gate.sample(None)
  assert not gate.yielding()
