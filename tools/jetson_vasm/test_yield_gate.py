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


def _logged_gate():
  clock, lines = Clock(), []
  return G.YieldGate(path='/nonexistent', clock=clock, start=False, log=lines.append), clock, lines


def _slow_pass(gate):
  for _ in range(20):
    gate.note(0, 3000)
  gate.note(0, 9000)


def test_an_ended_yield_is_logged_once_with_its_cause_and_length():
  gate, clock, lines = _logged_gate()
  _slow_pass(gate)
  clock.t += 2.0
  gate.note(0, 9000)              # extends: HOLD_S from here
  gate.tick()
  assert lines == []              # still yielding
  clock.t += G.HOLD_S + 0.5
  gate.tick()
  gate.tick()
  assert lines == [f'yielded {2.0 + G.HOLD_S:.1f} s: started by slow: pass 9000 us vs 3000 us quiet; extended by 1 slow']


def test_logging_does_not_change_the_decision():
  quiet, _ = _gate()
  logged, clock, _ = _logged_gate()
  quiet.clock = clock
  for gate in (quiet, logged):
    _slow_pass(gate)
  for _ in range(200):
    clock.t += 0.05
    logged.tick(0.5)
    assert quiet.yielding() == logged.yielding()


def test_the_summary_splits_an_open_yield_across_windows():
  gate, clock, lines = _logged_gate()
  _slow_pass(gate)                    # yields at t=100 until 105
  clock.t += G.SUMMARY_S - 2.0        # the window started at 100; the yield ended at 105
  gate.tick(0.4)
  clock.t += 2.0
  gate.note(0, 9000)                  # a new yield, open as the window closes at 160
  clock.t += 1.0
  gate.tick(0.6)
  summary = lines[-1]
  assert summary.startswith(f'yielding {(G.HOLD_S + 1.0) / (G.SUMMARY_S + 1.0):.0%} of the last 61 s')
  assert 'mean 50% peak 60%' in summary and f'ended yields (s) [{G.HOLD_S}]' in summary
  clock.t += G.HOLD_S
  gate.tick()
  assert lines[-1].startswith(f'yielded {G.HOLD_S:.1f} s')   # the whole episode, not the part after the summary
  assert gate._window['yield_s'] == G.HOLD_S - 1.0           # only its part after the summary counts again
