from io import BytesIO
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from PIL import Image
import numpy as np

from openpilot.starpilot.galaxy.camera_snapshot import CameraSnapshot, SnapshotDenied, SnapshotUnavailable
from openpilot.starpilot.galaxy.camera_snapshot_worker import brighten, capture


class CameraSnapshotTest(unittest.TestCase):
  def setUp(self):
    for name in ("request", "mark_frame"):
      replacement = patch("openpilot.starpilot.galaxy.camera_request." + name)
      setattr(self, name + "_mock", replacement.start())
      self.addCleanup(replacement.stop)

  def test_denied_does_not_launch_reader(self):
    owner = CameraSnapshot(spawn=lambda *args, **kwargs: self.fail("launched"))
    with self.assertRaises(SnapshotDenied):
      owner.capture('cabin', permitted=lambda: False)
    with self.assertRaises(ValueError):
      owner.capture('../camera', permitted=lambda: True)

  def test_revocation_kills_and_reaps_blocked_reader(self):
    children = []
    def spawn(args, **kwargs):
      self.assertEqual(args[-1], 'cabin')
      with self.assertRaises(SnapshotUnavailable):
        CameraSnapshot(spawn=lambda *a, **k: self.fail("concurrent reader")).capture(
          'wide', permitted=lambda: True)
      children.append(subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], **kwargs))
      return children[-1]
    calls = 0
    def permitted():
      nonlocal calls
      calls += 1
      return calls < 3
    with self.assertRaises(SnapshotDenied):
      CameraSnapshot(spawn=spawn).capture('cabin', permitted=permitted)
    self.assertIsNotNone(children[0].poll())
    self.assertTrue(children[0].stdout.closed)

  def test_deadline_kills_blocked_native_reader(self):
    children = []
    ticks = iter(range(100))
    def spawn(args, **kwargs):
      children.append(subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], **kwargs))
      return children[-1]
    with self.assertRaises(SnapshotUnavailable):
      CameraSnapshot(spawn=spawn, clock=lambda: next(ticks)).capture('cabin', permitted=lambda: True)
    self.assertIsNotNone(children[0].poll())
    self.assertTrue(children[0].stdout.closed)

  def test_reader_result_and_final_authority_check(self):
    def spawn(args, **kwargs):
      return subprocess.Popen([sys.executable, '-c', "import sys; sys.stdout.buffer.write(bytes([255,216,255,217]))"], **kwargs)
    owner = CameraSnapshot(spawn=spawn)
    self.assertEqual(owner.capture('wide', permitted=lambda: True), b'\xff\xd8\xff\xd9')
    self.assertEqual(owner.capture('cabin', permitted=lambda: True), b'\xff\xd8\xff\xd9')
    self.assertEqual(self.request_mock.call_count, 2)
    self.assertEqual(self.mark_frame_mock.call_count, 1) # Cabin availability receipt only.
    with self.assertRaises(SnapshotUnavailable):
      owner.capture('wide', permitted=lambda: True)
    calls = 0
    def permitted():
      nonlocal calls
      calls += 1
      return calls < 3
    with self.assertRaises(SnapshotDenied):
      CameraSnapshot(spawn=spawn).capture('wide', permitted=permitted)

  def test_real_nv12_conversion_accepts_only_post_request_fresh_native_frame(self):
    now = 1_000_000_000
    data = np.full(80, 128, dtype=np.uint8)
    frame = SimpleNamespace(width=4, height=4, stride=4, uv_offset=16, data=data)
    class Client:
      valid = False  # Native camerad leaves the extra metadata bit unset.
      timestamp_sof = now - 1
      timestamp_eof = now - 1
      frames = 0
      def connect(self, blocking):
        self.blocking = blocking
        return True
      def recv(self, timeout_ms):
        nonlocal now
        self.frames += 1
        now += 100_000_000
        if self.frames > 1:
          self.timestamp_sof = now - 20_000_000
          self.timestamp_eof = now - 10_000_000
        return frame
    client = Client()
    result = capture('cabin', client_factory=lambda *args: client, clock=lambda: now)
    self.assertFalse(client.blocking)
    self.assertEqual(client.frames, 100)
    image = Image.open(BytesIO(result))
    self.assertEqual(image.size, (4, 4))
    self.assertEqual(image.mode, 'RGB')
    frame.width = 99999
    self.assertIsNone(capture('cabin', client_factory=lambda *args: client, clock=lambda: now))


  def test_native_frames_with_stale_future_or_reversed_timestamps_never_convert(self):
    for fault in ('stale', 'future', 'reversed'):
      with self.subTest(fault=fault):
        now = [1_000_000_000]
        frame = SimpleNamespace(width=4, height=4, stride=4, uv_offset=16,
                                data=np.full(80, 128, dtype=np.uint8))
        class Client:
          valid = False
          timestamp_sof = timestamp_eof = 0
          def connect(self, blocking): return True
          def recv(self, timeout_ms, *, fault=fault, frame=frame, now=now):
            now[0] += 100_000_000
            self.timestamp_sof = now[0] - (600_000_000 if fault == 'stale' else 20_000_000)
            self.timestamp_eof = now[0] + 1 if fault == 'future' else self.timestamp_sof - 1 if fault == 'reversed' else now[0] - 10_000_000
            return frame
        with patch('openpilot.starpilot.galaxy.camera_snapshot_worker.extract_image') as convert:
          self.assertIsNone(capture('cabin', client_factory=lambda *args: Client(), clock=lambda now=now: now[0]))
          convert.assert_not_called()


class RoadSnapshotTest(unittest.TestCase):
  def _wide(self, exposure_factory, level=128, conversions=None):
    now = [1_000_000_000]
    frames = [SimpleNamespace(width=4, height=4, stride=4, uv_offset=16, data=np.full(80, level + step, dtype=np.uint8))
              for step in (-10, 10)]
    converted_at = []
    class Client:
      timestamp_sof = timestamp_eof = 0
      def connect(self, blocking): return True
      def recv(self, timeout_ms):
        now[0] += 50_000_000
        self.timestamp_sof, self.timestamp_eof = now[0] - 20_000_000, now[0] - 10_000_000
        return frames[now[0] // 50_000_000 % 2]
    def convert(buf):
      converted_at.append(now[0])
      if conversions is not None:
        conversions.append(int(np.asarray(buf.data)[0]))
      return np.full((4, 4, 3), level, dtype=np.uint8)
    with patch('openpilot.starpilot.galaxy.camera_snapshot_worker.extract_image', side_effect=convert):
      result = capture('wide', client_factory=lambda *args: Client(), clock=lambda: now[0], exposure_factory=exposure_factory)
    self.assertIsNotNone(result)
    return (converted_at[0] - 1_000_000_000) / 1e9

  def _watch(self, greys):
    from openpilot.starpilot.galaxy.camera_snapshot_worker import ExposureWatch
    states = iter(greys)
    class Messages(dict):
      updated = {'wideRoadCameraState': True}
      def update(self, timeout):
        self['wideRoadCameraState'] = SimpleNamespace(targetGreyFraction=0.2, measuredGreyFraction=next(states, 0.05))
    watch = ExposureWatch.__new__(ExposureWatch)
    watch.service, watch.messages, watch.run = 'wideRoadCameraState', Messages(), 0
    return watch

  def test_bright_scene_is_taken_once_exposure_reaches_target(self):
    waited = self._wide(lambda camera: self._watch([0.02, 0.08, 0.15, 0.19, 0.21, 0.2]))
    self.assertLess(waited, 0.5)

  def test_single_on_target_frame_is_not_enough(self):
    self.assertGreaterEqual(self._wide(lambda camera: self._watch([0.05, 0.2, 0.05, 0.2, 0.2, 0.05])), 4.0)

  def test_dark_scene_waits_the_full_settle_time(self):
    self.assertGreaterEqual(self._wide(lambda camera: self._watch([0.02] * 500)), 4.0)

  def test_dark_scene_averages_raw_frames_and_bright_scene_takes_one(self):
    dark, bright = [], []
    self._wide(lambda camera: None, level=30, conversions=dark)
    self._wide(lambda camera: None, level=120, conversions=bright)
    self.assertEqual(dark, [30])  # frames alternate 20/40; one conversion of their average
    self.assertEqual(len(bright), 1)
    self.assertIn(bright[0], (110, 130))

  def test_no_camera_state_falls_back_to_the_timer(self):
    self.assertGreaterEqual(self._wide(lambda camera: None), 4.0)

  def test_brighten_lifts_a_dark_scene_and_caps_gain_on_a_flat_one(self):
    rng = np.random.default_rng(0)
    dark = rng.integers(10, 80, size=(40, 40, 3)).astype(np.uint8)
    self.assertGreater(np.median(brighten(dark)), 2 * np.median(dark))
    flat = np.full((40, 40, 3), 25, dtype=np.uint8)
    flat[0, 0] = 27
    self.assertLessEqual(int(brighten(flat).max()) - int(brighten(flat).min()), 16 * 2 + 1)
    bright = rng.integers(0, 256, size=(40, 40, 3)).astype(np.uint8)
    self.assertLess(np.abs(brighten(bright).astype(int) - bright).mean(), 20)
