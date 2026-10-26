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


def _stats(p99, slow=0, t=1000.0):
  return {'event': 'stats', 'served_ms': {'p99': p99, 'mean': p99 - 0.1, 'max': p99 + 0.1}, 'slow': slow, 't': t}


def test_driving_model_frames_near_budget_yield_briefly():
  gate, clock = _gate()
  gate.on_stats(_stats(18.3), now_epoch=1000.5)
  assert gate.latency_live() and not gate.yielding()
  gate.on_stats(_stats(G.P99_MAX_MS + 1), now_epoch=1000.5)
  assert gate.yielding() and gate.stats['latency_yields'] == 1
  clock.t += G.LATENCY_HOLD_S + 0.01
  assert not gate.yielding()
  gate.on_stats(_stats(20.0, slow=1), now_epoch=1000.5)   # a frame Jetlink counted slow
  assert gate.yielding() and gate.stats['latency_yields'] == 2


def test_with_live_stats_load_and_slow_passes_do_not_yield():
  gate, clock = _gate()
  for _ in range(20):
    gate.note(0, 3000)
  for _ in range(100):
    clock.t += 0.05
    gate.on_stats(_stats(18.3), now_epoch=1000.5)
    gate.sample(1.0)
    gate.note(0, 9000)
  assert not gate.yielding() and gate.stats == {'load_yields': 0, 'slow_yields': 0, 'latency_yields': 0}


def test_stale_stats_fall_back_to_load_and_slow_passes():
  gate, clock = _gate()
  gate.on_stats(_stats(18.3), now_epoch=1000.5)
  clock.t += G.STATS_FRESH_S
  assert not gate.latency_live()
  for _ in range(20):
    gate.note(0, 3000)
  gate.note(0, 9000)
  assert gate.yielding() and gate.stats['slow_yields'] == 1


def test_replayed_and_malformed_stats_are_ignored():
  gate, _ = _gate()
  gate.on_stats(_stats(99.0, t=1000.0), now_epoch=1000.0 + G.STATS_FRESH_S + 1)   # from the two-minute replay
  gate.on_stats({'event': 'stats', 't': 1000.0}, now_epoch=1000.5)
  assert not gate.latency_live() and not gate.yielding()


def test_no_password_file_starts_no_stats_thread(tmp_path):
  import threading
  gate = G.YieldGate(path='/nonexistent', start=True, password_file=str(tmp_path / 'missing'))
  assert not gate.latency_live()
  assert 'yield-gate-stats' not in {t.name for t in threading.enumerate()}
