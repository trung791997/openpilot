"""Short parked Sentry clips (wide and cabin) from the livestream encoder, recorded in a bounded subprocess."""

import subprocess
import sys
import time

from openpilot.starpilot.galaxy.camera_snapshot import _CAPTURE_LOCK, SnapshotDenied, SnapshotUnavailable

MAX_CLIP_BYTES = 12_000_000  # the worker stops at 11.9 MB of frames; the MP4 boxes add a little
DEADLINE_MARGIN_SECONDS = 25  # worker gives up 19 s past the clip length; the encoder needs a few seconds to start
CLIP_SECONDS = (5, 10, 15)
RENEW_SECONDS = 5


def valid_clip(body) -> bool:
  return isinstance(body, bytes) and 64 <= len(body) <= MAX_CLIP_BYTES and body[4:8] == b"ftyp"


class CameraClip:
  def __init__(self, *, spawn=subprocess.Popen, clock=time.monotonic):
    self.spawn, self.clock = spawn, clock

  def capture(self, *, permitted, seconds=5) -> dict:
    if seconds not in CLIP_SECONDS:
      raise ValueError("Unsupported clip length")
    if not permitted():
      raise SnapshotDenied
    if not _CAPTURE_LOCK.acquire(blocking=False):
      raise SnapshotUnavailable("Another capture is in progress")
    process = None
    try:
      from openpilot.starpilot.galaxy.camera_request import request, request_clip
      from openpilot.starpilot.galaxy.camera_clip_worker import decode
      deadline = self.clock() + DEADLINE_MARGIN_SECONDS + seconds
      renewed = float('-inf')
      process = self.spawn([sys.executable, '-m', 'openpilot.starpilot.galaxy.camera_clip_worker', str(seconds)],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, close_fds=True)
      while self.clock() < deadline:
        if not permitted():
          raise SnapshotDenied
        if self.clock() - renewed >= RENEW_SECONDS:
          # Manager keeps camerad and stream_encoderd up while both leases hold, and stops them once they lapse.
          request()
          request_clip()
          renewed = self.clock()
        try:
          body, _ = process.communicate(timeout=min(.1, max(.001, deadline - self.clock())))
        except subprocess.TimeoutExpired:
          continue
        if not permitted():
          raise SnapshotDenied
        try:
          clips = decode(body) if process.returncode == 0 else {}
        except ValueError:
          clips = {}
        clips = {camera: clip for camera, clip in clips.items() if valid_clip(clip)}
        if not clips:
          raise SnapshotUnavailable("No camera clip")
        return clips
      raise SnapshotUnavailable("Camera clip timed out")
    except OSError as error:
      raise SnapshotUnavailable("Camera unavailable") from error
    finally:
      if process is not None:
        if process.poll() is None:
          process.kill()
        process.wait()
        if process.stdout is not None:
          process.stdout.close()
      _CAPTURE_LOCK.release()
