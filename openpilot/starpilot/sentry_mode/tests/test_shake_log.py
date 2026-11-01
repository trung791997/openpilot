"""The parked shake log is observation only: right counts, bounded storage, and no effect on detection."""

import importlib.util
import json
import os
from pathlib import Path
import stat
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.starpilot.sentry_mode import shake_log as shake_module
from openpilot.starpilot.sentry_mode.policy import MotionSample, Settings
from openpilot.starpilot.sentry_mode.preferences import KEY, Preferences, encode
from openpilot.starpilot.sentry_mode.runtime import SentryRuntime
from openpilot.starpilot.sentry_mode.shake_log import THRESHOLDS, ShakeLog
from openpilot.starpilot.sentry_mode.tests.test_runtime import Clock, Messages

REPORT = Path(__file__).resolve().parents[4] / "tools/sentry_shake_report.py"
DAY_NS = 86_400_000_000_000


def read_lines(root: Path) -> list[dict]:
  return [json.loads(raw) for file in sorted(root.glob("*.jsonl")) for raw in file.read_text().splitlines()]


class WallClock:
  def __init__(self, ns=1_793_500_000_000_000_000):
    self.ns = ns

  def __call__(self):
    return self.ns


class ShakeLogUnitTest(unittest.TestCase):
  def setUp(self):
    temporary = tempfile.TemporaryDirectory()
    self.addCleanup(temporary.cleanup)
    self.root = Path(temporary.name) / "sentry/shake"
    self.wall = WallClock()

  def feed(self, logger, magnitudes, *, start_ns=10_000_000_000, state="armed", event_at=None, trig=0):
    policy = NS(settings=Settings(0.08, 3.0), trigger_count=trig)
    for index, magnitude in enumerate(magnitudes):
      now = start_ns + index * 100_000_000
      event = "warning" if index == event_at else None
      logger.observe(now, MotionSample(now / 1e9, (0.0, 0.0, magnitude)), NS(state=state, event=event), policy, event is not None)
      self.wall.ns += 100_000_000

  def test_threshold_counts_max_sum_and_signed_deltas(self):
    logger = ShakeLog(self.root, wall_clock=self.wall, background=False)
    deltas = [0.025, -0.035, 0.05, -0.07, 0.09, -0.11, 0.13, -0.16, 0.25, -0.31]
    magnitudes = [9.81]
    for delta in deltas:
      magnitudes.append(magnitudes[-1] + delta)
    # The first sample at 19.9 s gives no delta; 10 deltas fill 20.0-20.9 s and a quiet sample closes the window.
    self.feed(logger, magnitudes + [magnitudes[-1]], start_ns=19_900_000_000, event_at=4, trig=7)
    logger.close()
    lines = read_lines(self.root)
    heartbeat, window = lines[0], lines[1]
    self.assertEqual(heartbeat["hb"], 1)
    self.assertEqual((heartbeat["sens"], heartbeat["warnN"]), (0.08, 30))
    sizes = [abs(d) for d in deltas]
    self.assertEqual(window["over"], [sum(size > t for size in sizes[:10]) for t in THRESHOLDS])
    self.assertEqual(window["over"], [10, 9, 8, 7, 6, 5, 4, 3, 2, 1])
    self.assertAlmostEqual(window["max"], 0.31, places=4)
    self.assertAlmostEqual(window["sum"], sum(sizes), places=4)
    self.assertEqual(window["d"], [round(d, 4) for d in deltas])
    self.assertEqual((window["n"], window["armed"], window["trig"]), (10, True, 7))
    self.assertEqual(window["events"], [{"kind": "warning", "monoNs": 20_300_000_000, "queued": True}])
    self.assertEqual(len(lines), 2)

  def test_quiet_seconds_write_nothing_but_heartbeats_every_ten_minutes(self):
    logger = ShakeLog(self.root, wall_clock=self.wall, background=False)
    self.feed(logger, [9.81 + (0.01 if i % 2 else 0.0) for i in range(12_001)], state="armed")  # 20 min, |delta| 0.01
    logger.close()
    lines = read_lines(self.root)
    self.assertEqual([line.get("hb") for line in lines], [1, 1, 1])
    self.assertTrue(all(line["armed"] for line in lines))

  def test_private_modes_cap_and_oldest_day_deleted_first(self):
    logger = ShakeLog(self.root, wall_clock=self.wall, max_bytes=6_000, background=False)
    for day in range(4):
      self.wall.ns = 1_793_500_000_000_000_000 + day * DAY_NS
      shaky = [9.81 + (0.1 if i % 2 else 0.0) for i in range(60)]
      self.feed(logger, shaky + [9.81], start_ns=(day + 1) * 1_000_000_000_000)
      logger.flush()
    files = sorted(self.root.glob("*.jsonl"))
    total = sum(f.stat().st_size for f in files)
    self.assertLessEqual(total, 6_000)
    self.assertGreater(total, 2_000)
    self.assertLess(len(files), 4)
    newest = shake_module.time.strftime("%Y-%m-%d", shake_module.time.gmtime(self.wall.ns // 1_000_000_000))
    self.assertEqual(files[-1].name, newest + ".jsonl")
    for file in files:
      self.assertEqual(stat.S_IMODE(file.stat().st_mode), 0o600)
    self.assertEqual(stat.S_IMODE(self.root.stat().st_mode), 0o700)

  def test_a_day_larger_than_the_cap_is_dropped_not_written(self):
    logger = ShakeLog(self.root, wall_clock=self.wall, max_bytes=500, background=False)
    self.feed(logger, [9.81 + (0.1 if i % 2 else 0.0) for i in range(60)] + [9.81])
    logger.flush()
    self.assertLessEqual(sum(f.stat().st_size for f in self.root.glob("*.jsonl")), 500)
    self.assertGreater(logger.dropped, 0)

  def test_unwritable_storage_is_swallowed_bounded_and_logged_once(self):
    self.root.parent.mkdir(parents=True)
    self.root.symlink_to(self.root.parent)  # Not a private directory: every write fails.
    logger = ShakeLog(self.root, wall_clock=self.wall, background=False)
    with patch("openpilot.common.swaglog.cloudlog.exception") as logged:
      for _ in range(3):
        self.feed(logger, [9.81 + (0.1 if i % 2 else 0.0) for i in range(30)] + [9.81])
        logger.flush()
      logger.close()
    self.assertEqual(logged.call_count, 1)  # Rate-limited to one per ten minutes.
    self.assertLessEqual(logger.buffer_bytes, shake_module.MAX_BUFFER_BYTES)

  def test_background_writer_flushes_on_close(self):
    logger = ShakeLog(self.root, wall_clock=self.wall)
    self.feed(logger, [9.81 + (0.1 if i % 2 else 0.0) for i in range(20)] + [9.81])
    self.assertIsNotNone(logger.thread)
    logger.close()
    self.assertFalse(logger.thread.is_alive())
    self.assertEqual(sum(1 for line in read_lines(self.root) if not line.get("hb")), 3)  # 10.1-12.0 s spans three seconds


class RuntimeHarness:
  """Drives a SentryRuntime through the same harness as test_runtime with a scripted magnitude stream."""

  def __init__(self, directory: str, shake=None):
    self.params = Params()
    self.clock = Clock()
    self.messages = Messages(self.clock)
    self.events = []
    self.runtime = SentryRuntime(self.params, self.messages, mono_clock=lambda: self.clock.mono, boot_clock=self.clock.boot,
                                 record=lambda kind, stamp, permitted: self.events.append((kind, stamp, permitted())),
                                 start=lambda target: target(), shake_log=shake)

  def tick(self, z):
    self.clock.mono += 100_000_000
    raw = self.messages.set_sources(acceleration=(0.0, 0.3, z))
    with log.Event.from_bytes(raw) as parsed:
      self.messages.messages["accelerometer"] = parsed.accelerometer
      return self.runtime.tick()


def stream():
  """Arming, quiet, rumble below 0.08, a 3 s get-in that warns, then 40 s of shaking that alarms, then quiet."""
  values = [9.81] * 905
  values += [9.81 + (0.05 if i % 2 else 0.0) for i in range(200)]
  values += [9.81] * 50
  values += [9.81 + (0.12 if i % 2 else 0.0) for i in range(35)]
  values += [9.81] * 100
  values += [9.81 + (0.2 if i % 2 else 0.0) for i in range(400)]
  values += [9.81] * 700
  return values


class DetectionUnchangedTest(unittest.TestCase):
  def setUp(self):
    self.directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.directory.cleanup)
    self.environment = patch.dict(os.environ, PARAMS_ROOT=self.directory.name, OPENPILOT_PREFIX="sentryshake",
                                  STARPILOT_SENTRY_DEVELOPMENT="1")
    self.environment.start()
    self.addCleanup(self.environment.stop)
    params = Params()
    params.put_bool("IsOffroad", True, block=True)
    Path(params.get_param_path(KEY)).write_bytes(encode(Preferences(True, Settings(0.08, 3.0))))
    self.shake_root = Path(self.directory.name) / "shake"

  def run_stream(self, shake):
    harness = RuntimeHarness(self.directory.name, shake)
    decisions = [harness.tick(z) for z in stream()]
    harness.runtime.close()
    return decisions, harness.events, harness.runtime.policy.trigger_count

  def test_decisions_and_events_identical_with_and_without_the_log(self):
    baseline = self.run_stream(None)
    self.assertEqual([kind for kind, _, _ in baseline[1]], ["warning", "alarm"])
    logged = self.run_stream(ShakeLog(self.shake_root, background=False))
    self.assertEqual(logged, baseline)

    class Broken(ShakeLog):
      def _observe(self, *args):
        raise RuntimeError("boom")

      def _flush(self):
        raise OSError("disk")

    with patch("openpilot.common.swaglog.cloudlog.exception") as logged_error:
      broken = self.run_stream(Broken(self.shake_root / "broken", background=False))
    self.assertEqual(broken, baseline)
    self.assertEqual(logged_error.call_count, 1)

    class Hostile:
      def observe(self, *args):
        raise RuntimeError("not a ShakeLog")

      def close(self):
        raise RuntimeError("not a ShakeLog")

    self.assertEqual(self.run_stream(Hostile()), baseline)

  def test_log_records_the_events_and_report_replay_matches_the_live_policy(self):
    decisions, events, _ = self.run_stream(ShakeLog(self.shake_root, background=False))
    lines = read_lines(self.shake_root)
    windows = [line for line in lines if not line.get("hb")]
    logged_events = [event for window in windows for event in window["events"]]
    self.assertEqual([(e["kind"], e["monoNs"], e["queued"]) for e in logged_events],
                     [(kind, stamp, True) for kind, stamp, _ in events])
    self.assertTrue(all(line["armed"] for line in windows))
    # The 0.05 rumble is logged, below Peter's 0.08: counted at 0.04 and 0.03 but not 0.08.
    rumble = windows[0]
    self.assertEqual(rumble["over"][THRESHOLDS.index(0.08)], 0)
    self.assertGreater(rumble["over"][THRESHOLDS.index(0.04)], 0)

    spec = importlib.util.spec_from_file_location("sentry_shake_report", REPORT)
    report = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(report)
    windows.sort(key=lambda w: w["monoNs"])
    replayed = [kind for stretch in report.segments(windows) for kind, _ in report.replay(stretch, Settings(0.08, 3.0))]
    self.assertEqual(replayed, [kind for kind, _, _ in events])
    # At the default 0.04 / 1 s the rumble warns as well, which is what Peter saw at home.
    looser = [kind for stretch in report.segments(windows) for kind, _ in report.replay(stretch, Settings(0.04, 1.0))]
    self.assertEqual(looser.count("warning"), 2)


if __name__ == "__main__":
  unittest.main()
