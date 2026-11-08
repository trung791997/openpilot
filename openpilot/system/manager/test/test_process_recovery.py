import unittest
from unittest.mock import patch

from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.system.manager import process


class Child:
  def __init__(self, **kwargs):
    self.exitcode: int | None = None
    self.pid = 123
    self.joined = False

  def start(self):
    pass

  def join(self):
    self.joined = True

  def is_alive(self):
    return self.exitcode is None


class TestProcessRecovery(unittest.TestCase):
  def setUp(self):
    self.enterContext(OpenpilotPrefix())
    self.params = Params()
    self.cp = car.CarParams.new_message(brand="hyundai", carFingerprint="HYUNDAI_IONIQ_6")
    self.now = 100.0
    self.enterContext(patch.object(process, "Process", Child))
    self.enterContext(patch.object(process.time, "monotonic", lambda: self.now))
    self.enterContext(patch.object(process, "cloudlog"))
    self.enterContext(patch.object(process.os, "kill"))
    self.enterContext(patch.object(process, "join_process", lambda p, timeout: setattr(p, "exitcode", 0)))

  def make_process(self, restart=True):
    return process.PythonProcess("ui", "ui", lambda started, params, cp: started, restart_on_exit=restart)

  def step(self, p, started=True, ignored=None):
    process.ensure_running({p.name: p}.values(), started, self.params, self.cp, not_run=ignored)

  def test_crash_restarts_after_delay_and_reaps_child(self):
    p = self.make_process()
    self.step(p)
    old = p.proc
    old.exitcode = 1
    self.step(p)
    self.assertIs(p.proc, old)
    self.now += 0.99
    self.step(p)
    self.assertIs(p.proc, old)
    self.now += 0.02
    self.step(p)
    self.assertIsNot(p.proc, old)
    self.assertTrue(old.joined)

  def test_backoff_caps_and_stable_run_resets(self):
    p = self.make_process()
    self.step(p)
    for delay in (1, 2, 4, 8, 16, 30, 30):
      p.proc.exitcode = 1
      self.step(p)
      self.assertEqual(p.restart_at - self.now, delay)
      self.now += delay
      self.step(p)
    self.now += 60
    self.step(p)
    self.assertEqual(p.restart_failures, 0)
    p.proc.exitcode = 0
    self.step(p)
    self.assertEqual(p.restart_at - self.now, 1)

  def test_gates_cancel_pending_restart(self):
    for reason in ("predicate", "disabled", "ignored"):
      with self.subTest(reason=reason):
        p = self.make_process()
        self.step(p)
        p.proc.exitcode = 1
        self.step(p)
        p.enabled = reason != "disabled"
        started = reason != "predicate"
        ignored = ["ui"] if reason == "ignored" else None
        self.step(p, started, ignored)
        self.assertIsNone(p.proc)
        self.assertEqual(p.restart_at, 0)
        self.now += 40
        self.step(p, started, ignored)
        self.assertIsNone(p.proc)

  def test_other_processes_do_not_restart(self):
    p = self.make_process(restart=False)
    self.step(p)
    old = p.proc
    old.exitcode = 1
    self.now += 100
    self.step(p)
    self.assertIs(p.proc, old)

  def test_intentional_stop_is_not_crash(self):
    p = self.make_process()
    self.step(p)
    p.stop(block=False)
    self.assertTrue(p.shutting_down)
    p.proc.exitcode = 0
    self.step(p)
    self.assertEqual(p.restart_failures, 0)
    self.assertEqual(p.restart_at, 0)

  def test_a_crashed_jetlink_owner_restarts(self):
    from openpilot.system.manager.process_config import managed_processes
    configured = managed_processes["jetlinkd"]
    self.assertTrue(configured.restart_on_exit)
    assert isinstance(configured, process.PythonProcess)
    p = process.PythonProcess("jetlinkd", "unused", lambda started, params, cp: True, restart_on_exit=configured.restart_on_exit)
    self.step(p)
    old = p.proc
    assert isinstance(old, Child)
    old.exitcode = 1
    self.step(p)
    self.assertIs(p.proc, old)
    self.now += 1.01
    self.step(p)
    self.assertIsNot(p.proc, old)
    self.assertTrue(old.joined)

  def test_learner_recovers_with_backoff_only_while_onroad(self):
    from openpilot.system.manager.process_config import managed_processes
    configured = managed_processes["steerlearnerd"]
    self.assertTrue(configured.restart_on_exit)
    p = process.PythonProcess(configured.name, configured.module, configured.should_run,
                              restart_on_exit=configured.restart_on_exit)
    self.step(p)
    old = p.proc
    self.assertIsNotNone(old)
    assert isinstance(old, Child)
    old.exitcode = 1
    self.step(p)
    self.assertIs(p.proc, old)
    self.now += 1.01
    self.step(p)
    self.assertIsNot(p.proc, old)
    self.assertTrue(old.joined)
    assert isinstance(p.proc, Child)
    p.proc.exitcode = 1
    self.step(p)
    self.step(p, started=False)
    self.now += 60
    self.step(p, started=False)
    self.assertIsNone(p.proc)
    self.assertEqual(p.restart_at, 0)

  def test_boot_racing_processes_restart(self):
    from openpilot.system.manager.process_config import procs
    restarting = {p.name for p in procs if getattr(p, "restart_on_exit", False)}
    self.assertLessEqual({"ui", "soundd", "starportd"}, restarting)


  def test_sensor_three_recovery_cycles_obey_backoff_and_stop_eligibility(self):
    from openpilot.system.manager import process_config
    configured = process_config.managed_processes['sensord']
    p = process.PythonProcess('sensord', configured.module, configured.should_run,
                             restart_on_exit=configured.restart_on_exit)
    with patch.object(process_config, 'sentry_enabled', return_value=False):
      self.step(p)
      for delay in (1, 2, 4):
        old = p.proc
        assert isinstance(old, Child)
        old.exitcode = 1
        self.step(p)
        self.assertEqual(p.restart_at - self.now, delay)
        self.now += delay - .01
        self.step(p)
        self.assertIs(p.proc, old)
        self.now += .02
        self.step(p)
        self.assertIsNot(p.proc, old)
        self.assertTrue(old.joined)
      self.step(p, started=False)
      self.assertTrue(p.shutting_down)
      stopping = p.proc
      self.step(p, started=False)
      self.assertIs(p.proc, stopping)
      assert isinstance(stopping, Child)
      stopping.exitcode = 0
      self.step(p, started=False)
      self.assertIsNone(p.proc)
      self.now += 60
      self.step(p, started=False)
      self.assertIsNone(p.proc)
      p.enabled = False
      self.step(p, started=True)
      self.assertIsNone(p.proc)


class TestEncoderHandoff(unittest.TestCase):
  """2026-11-07: stream_encoderd was still closing when encoderd started at ignition; encoderd aborted for the drive."""
  setUp = TestProcessRecovery.setUp
  step = TestProcessRecovery.step

  def make_pair(self):
    enc = process.NativeProcess("encoderd", ".", ["unused"], lambda started, params, cp: started,
                                restart_on_exit=True, wait_for=("stream_encoderd",))
    self.clip = True
    stream = process.NativeProcess("stream_encoderd", ".", ["unused"], lambda started, params, cp: self.clip and not started,
                                   wait_for=("encoderd",))
    return enc, stream

  def step_all(self, ps, started):
    process.ensure_running({p.name: p for p in ps}.values(), started, self.params, self.cp)

  def test_encoderd_waits_for_closing_stream_encoder(self):
    enc, stream = self.make_pair()
    self.enterContext(patch.object(process, "join_process", lambda p, timeout: None))
    self.step_all([enc, stream], False)
    self.assertIsNotNone(stream.proc)
    self.step_all([enc, stream], True)
    self.assertTrue(stream.shutting_down)
    self.assertIsNone(enc.proc)
    stream.proc.exitcode = -2
    self.step_all([enc, stream], True)
    self.assertIsNone(stream.proc)
    self.assertIsNotNone(enc.proc)

  def test_live_stream_encoder_does_not_block(self):
    enc, stream = self.make_pair()
    stream.should_run = lambda *_: True
    self.step_all([enc, stream], True)
    self.assertIsNotNone(enc.proc)
    self.assertIsNotNone(stream.proc)

  def test_native_encoderd_restarts_after_crash(self):
    enc, _ = self.make_pair()
    self.step(enc)
    old = enc.proc
    old.exitcode = -6
    self.step(enc)
    self.assertIs(enc.proc, old)
    self.now += 1.01
    self.step(enc)
    self.assertIsNot(enc.proc, old)
    self.assertTrue(old.joined)

  def test_native_without_restart_stays_down(self):
    _, stream = self.make_pair()
    self.step(stream, started=False)
    old = stream.proc
    old.exitcode = -6
    self.now += 60
    self.step(stream, started=False)
    self.assertIs(stream.proc, old)
