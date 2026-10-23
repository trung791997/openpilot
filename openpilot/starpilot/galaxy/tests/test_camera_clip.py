import shutil
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from openpilot.starpilot.galaxy import camera_request, mp4
from openpilot.starpilot.galaxy.camera_clip import CameraClip, valid_clip
from openpilot.starpilot.galaxy.camera_clip_worker import KEYFRAME, Track, decode, encode, record
from openpilot.starpilot.galaxy.camera_snapshot import SnapshotDenied, SnapshotUnavailable

SPS = b"\x67\x64\x00\x1f\xac\xd9"
PPS = b"\x68\xeb\xe3\xcb"
FRAME_NS = 50_000_000


def annexb(*units):
  return b"".join(b"\x00\x00\x00\x01" + unit for unit in units)


def message(index, *, key=None, header=True, width=1152, height=720):
  key = index % 5 == 0 if key is None else key
  data = annexb(b"\x65" + bytes([index % 255 + 1]) * 40) if key else annexb(b"\x41" + bytes([index % 255 + 1]) * 20)
  return SimpleNamespace(idx=SimpleNamespace(flags=KEYFRAME if key else 0, timestampEof=index * FRAME_NS),
                         data=data, header=annexb(SPS, PPS) if key and header else b"", width=width, height=height)


def boxes(body, offset=0, end=None):
  end = len(body) if end is None else end
  result = []
  while offset < end:
    size, kind = struct.unpack_from(">I4s", body, offset)
    result.append((kind, offset, size))
    offset += size
  return result


def child(body, *path):
  offset, end = 0, len(body)
  for kind in path:
    skip = 8 + {b"stsd": 8}.get(kind, 0)
    found = {k: (o, s) for k, o, s in boxes(body, offset, end)}[kind]
    offset, end = found[0] + skip, found[0] + found[1]
  return body[offset - 8:end] if path[-1] != b"stsd" else body[offset - 16:end]


class Mp4Test(unittest.TestCase):
  def frames(self, count):
    return [(m.idx.timestampEof, bool(m.idx.flags & KEYFRAME), bytes(m.data)) for m in map(message, range(count))]

  def test_parameter_sets_and_nal_split(self):
    self.assertEqual(mp4.nal_units(annexb(SPS, PPS) + b"\x00\x00\x01\x41\x01\x00"), [SPS, PPS, b"\x41\x01"])
    self.assertEqual(mp4.parameter_sets(annexb(SPS), annexb(PPS)), (SPS, PPS))
    self.assertIsNone(mp4.parameter_sets(annexb(SPS)))

  def test_boxes_index_every_frame_from_first_keyframe(self):
    frames = self.frames(23)
    body = mp4.mux(1152, 720, SPS, PPS, [(0, False, annexb(b"\x41\x00"))] + frames)
    self.assertEqual([kind for kind, _, _ in boxes(body)], [b"ftyp", b"moov", b"mdat"])  # moov first: playable while downloading
    stbl = (b"moov", b"trak", b"mdia", b"minf", b"stbl")
    sizes = child(body, *stbl, b"stsz")
    self.assertEqual(struct.unpack_from(">I", sizes, 16)[0], 23)
    keys = child(body, *stbl, b"stss")
    count = struct.unpack_from(">I", keys, 12)[0]
    self.assertEqual(struct.unpack_from(f">{count}I", keys, 16), (1, 6, 11, 16, 21))
    times = child(body, *stbl, b"stts")
    self.assertEqual(struct.unpack_from(">III", times, 12), (1, 23, 4500))  # 20 Hz at 90 kHz
    offset = struct.unpack_from(">I", child(body, *stbl, b"stco"), 16)[0]
    first = frames[0][2][4:]
    self.assertEqual(body[offset:offset + 4 + len(first)], struct.pack(">I", len(first)) + first)
    self.assertEqual(body[offset - 4:offset], b"mdat")
    with self.assertRaises(ValueError):
      mp4.mux(1152, 720, SPS, PPS, [(0, False, annexb(b"\x41\x00"))])

  @unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"), "needs ffmpeg")
  def test_real_encoder_stream_decodes_to_identical_frames(self):
    with tempfile.TemporaryDirectory() as directory:
      raw = f"{directory}/clip.h264"
      subprocess.run(["ffmpeg", "-y", "-nostdin", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc2=size=320x192:rate=20",
                      "-frames:v", "40", "-c:v", "libx264", "-bf", "0", "-g", "5", "-pix_fmt", "yuv420p", raw], check=True)
      units = mp4.nal_units(open(raw, "rb").read())
      frames, pending, stamp = [], [], 0
      for unit in units:
        pending.append(unit)
        if unit[0] & 0x1F in (1, 5):
          frames.append((stamp, unit[0] & 0x1F == 5, annexb(*pending)))
          pending, stamp = [], stamp + FRAME_NS
      out = f"{directory}/clip.mp4"
      with open(out, "wb") as f:
        f.write(mp4.mux(320, 192, *mp4.parameter_sets(frames[0][2]), frames))
      probe = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "stream=codec_name,nb_frames,duration",
                              "-of", "csv=p=0", out], capture_output=True, text=True, check=True).stdout.strip()
      self.assertEqual(probe, "h264,2.000000,40")
      hashes = [subprocess.run(["ffmpeg", "-nostdin", "-loglevel", "error", "-i", path, "-f", "framemd5", "-"],
                               capture_output=True, text=True, check=True).stdout.splitlines()[-40:] for path in (raw, out)]
      self.assertEqual([line.split(",")[-1] for line in hashes[0]], [line.split(",")[-1] for line in hashes[1]])


class WorkerTest(unittest.TestCase):
  def test_track_waits_for_settle_then_keyframe_and_stops_at_five_seconds(self):
    exposure = SimpleNamespace(settled=lambda: False)
    track = Track(1_000_000_000, exposure)
    now = 0
    for index in range(200):
      track.add(now, message(index))
      now += FRAME_NS
    self.assertTrue(track.done)
    self.assertEqual(track.frames[0][0], 20 * FRAME_NS)  # settled at 1 s, which is a keyframe
    self.assertEqual(len(track.frames), 5_000_000_000 // FRAME_NS)
    self.assertTrue(valid_clip(track.clip()))

  def test_track_runs_the_chosen_length(self):
    track = Track(0, SimpleNamespace(settled=lambda: True), 15_000_000_000)
    for index in range(400):
      track.add(index * FRAME_NS, message(index))
    self.assertTrue(track.done)
    self.assertEqual(len(track.frames), 15_000_000_000 // FRAME_NS)

  def test_track_starts_early_when_exposure_settles_and_needs_header(self):
    track = Track(10_000_000_000, SimpleNamespace(settled=lambda: True))
    track.add(0, message(0, header=False))
    track.add(0, message(1))
    self.assertEqual(track.frames, [])
    track.add(0, message(5))
    self.assertEqual(len(track.frames), 1)
    self.assertIsNone(track.clip())  # under a second of video is not sent

  def test_record_both_cameras_and_framing(self):
    clock = SimpleNamespace(now=0)
    queues = {"wide": iter(range(400)), "cabin": iter(range(400))}
    def receive(camera):
      clock.now += FRAME_NS // 2
      return [message(next(queues[camera]))]
    clips = record(receive, clock=lambda: clock.now, sleep=lambda _: None,
                   exposure_factory=lambda _: SimpleNamespace(settled=lambda: True))
    self.assertEqual(set(clips), {"wide", "cabin"})
    self.assertEqual(decode(encode(clips)), clips)
    for raw in (b"\x00\x00", b"\x02\x00\x00\x00\x00", encode({"wide": b"x"}) * 2, encode({"wide": b"xy"})[:-1]):
      with self.assertRaises(ValueError):
        decode(raw)

  def test_record_gives_up_at_deadline_without_frames(self):
    ticks = iter(range(0, 10**12, 10**9))
    self.assertEqual(record(lambda camera: [], clock=lambda: next(ticks), sleep=lambda _: None,
                            exposure_factory=lambda _: None), {})


class ClipLeaseTest(unittest.TestCase):
  def test_clip_lease_is_separate_and_expires(self):
    with tempfile.TemporaryDirectory() as directory:
      lease = f"{directory}/clip-lease"
      self.assertFalse(camera_request.clip_requested(lambda: 0, lease))
      camera_request.request_clip(lambda: 0, lease)
      self.assertTrue(camera_request.clip_requested(lambda: 1, lease))
      self.assertFalse(camera_request.clip_requested(lambda: camera_request.TTL_NS, lease))


class CameraClipTest(unittest.TestCase):
  def setUp(self):
    for name in ("request", "request_clip"):
      replacement = patch("openpilot.starpilot.galaxy.camera_request." + name)
      setattr(self, name + "_mock", replacement.start())
      self.addCleanup(replacement.stop)

  @staticmethod
  def writer(output):
    def spawn(args, **kwargs):
      return subprocess.Popen([sys.executable, '-c', f"import sys; sys.stdout.buffer.write({output!r})"], **kwargs)
    return spawn

  def test_valid_output_holds_both_leases(self):
    clip = bytes(4) + b"ftyp" + bytes(100)
    clips = CameraClip(spawn=self.writer(encode({"wide": clip, "cabin": b"not a clip"}))).capture(permitted=lambda: True)
    self.assertEqual(clips, {"wide": clip})
    self.assertTrue(self.request_mock.called and self.request_clip_mock.called)

  def test_length_reaches_the_worker_and_odd_lengths_are_refused(self):
    launched = []
    def spawn(args, **kwargs):
      launched.append(args)
      return self.writer(encode({"wide": bytes(4) + b"ftyp" + bytes(100)}))(args, **kwargs)
    CameraClip(spawn=spawn).capture(permitted=lambda: True, seconds=15)
    self.assertEqual(launched[0][-1], "15")
    with self.assertRaises(ValueError):
      CameraClip(spawn=lambda *a, **k: self.fail("launched")).capture(permitted=lambda: True, seconds=30)

  def test_denied_empty_and_revoked(self):
    with self.assertRaises(SnapshotDenied):
      CameraClip(spawn=lambda *a, **k: self.fail("launched")).capture(permitted=lambda: False)
    with self.assertRaises(SnapshotUnavailable):
      CameraClip(spawn=self.writer(b"")).capture(permitted=lambda: True)
    children = []
    def spawn(args, **kwargs):
      children.append(subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], **kwargs))
      return children[-1]
    calls = 0
    def permitted():
      nonlocal calls
      calls += 1
      return calls < 4
    with self.assertRaises(SnapshotDenied):
      CameraClip(spawn=spawn).capture(permitted=permitted)
    self.assertIsNotNone(children[0].poll())
    ticks = iter(range(0, 1000, 5))
    with self.assertRaises(SnapshotUnavailable):
      CameraClip(spawn=spawn, clock=lambda: next(ticks)).capture(permitted=lambda: True)
    self.assertIsNotNone(children[1].poll())


if __name__ == "__main__":
  unittest.main()
