"""Parked Sentry clips from the livestream encoder (stream_encoderd), repacked to MP4. Run as a subprocess.

The wide clip waits for auto exposure like the still picture (at most 4 s); the cabin clip waits the cabin's 10 s
settle. Each clip starts on a keyframe (every 5th frame) and runs 5 s. The encoder sees single frames, so night clips
get none of the still picture's frame averaging: expect them darker and grainier than night pictures.
"""
import struct
import sys
import time

from openpilot.starpilot.galaxy import mp4
from openpilot.starpilot.galaxy.camera_snapshot_worker import CABIN_SETTLE_NS, ROAD_SETTLE_NS, _exposure_watch, boot_ns

CAMERAS = ("wide", "cabin")
SERVICES = {"wide": "livestreamWideRoadEncodeData", "cabin": "livestreamCabinEncodeData"}
CLIP_NS = 5_000_000_000
MIN_FRAMES = 20  # 1 s at 20 Hz; anything shorter is not worth sending
MAX_CLIP_BYTES = 4_500_000  # 5 s at the 5 Mbps stream bitrate is ~3.1 MB; two clips stay under Discord's 10 MB
DEADLINE_NS = 24_000_000_000
KEYFRAME = 0x8  # V4L2_BUF_FLAG_KEYFRAME


class Track:
  def __init__(self, settle_ns, exposure=None):
    self.settle_ns, self.exposure = settle_ns, exposure
    self.first = None
    self.frames = []
    self.header = b""
    self.width = self.height = self.size = 0
    self.done = False

  def add(self, now, message):
    if self.done:
      return
    self.first = now if self.first is None else self.first
    key = bool(message.idx.flags & KEYFRAME)
    if not self.frames:
      settled = self.exposure is not None and self.exposure.settled()
      if (not settled and now - self.first < self.settle_ns) or not key or not message.header:
        return
      self.header, self.width, self.height = bytes(message.header), message.width, message.height
    stamp, data = message.idx.timestampEof, bytes(message.data)
    if self.frames and stamp - self.frames[0][0] >= CLIP_NS or self.size + len(data) > MAX_CLIP_BYTES:
      self.done = True
      return
    self.frames.append((stamp, key, data))
    self.size += len(data)

  def clip(self):
    if len(self.frames) < MIN_FRAMES or not 0 < self.width <= 4096 or not 0 < self.height <= 4096:
      return None
    sets = mp4.parameter_sets(self.header, self.frames[0][2])
    return mp4.mux(self.width, self.height, *sets, self.frames) if sets is not None else None


def record(receive, *, clock=boot_ns, sleep=time.sleep, exposure_factory=_exposure_watch):
  """receive(camera) -> EncodeData messages since the last call. Both cameras record at once."""
  started = clock()
  tracks = {"wide": Track(ROAD_SETTLE_NS, exposure_factory("wide")), "cabin": Track(CABIN_SETTLE_NS)}
  while clock() - started < DEADLINE_NS and not all(track.done for track in tracks.values()):
    received = False
    for camera, track in tracks.items():
      for message in receive(camera):
        received = True
        track.add(clock(), message)
    if not received:
      sleep(.02)
  clips = {}
  for camera, track in tracks.items():
    clip = track.clip()
    if clip is not None:
      clips[camera] = clip
  return clips


def encode(clips):
  return b"".join(struct.pack(">BI", CAMERAS.index(camera), len(body)) + body for camera, body in clips.items())


def decode(raw):
  clips, offset = {}, 0
  while offset < len(raw):
    if len(raw) - offset < 5:
      raise ValueError("Truncated clip output")
    index, size = struct.unpack_from(">BI", raw, offset)
    body = raw[offset + 5:offset + 5 + size]
    if index >= len(CAMERAS) or len(body) != size or CAMERAS[index] in clips:
      raise ValueError("Invalid clip output")
    clips[CAMERAS[index]] = body
    offset += 5 + size
  return clips


def main():
  from openpilot.cereal import messaging
  sockets = {camera: messaging.sub_sock(service, conflate=False) for camera, service in SERVICES.items()}
  def receive(camera):
    return [getattr(event, SERVICES[camera]) for event in messaging.drain_sock(sockets[camera])]
  clips = record(receive)
  if not clips:
    sys.exit(1)
  sys.stdout.buffer.write(encode(clips))


if __name__ == '__main__':
  try:
    main()
  except Exception:
    sys.exit(1)
