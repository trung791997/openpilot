"""CPU-only frame reader. Does not start or stop camerad or change Params."""

from io import BytesIO
import sys
import time
from types import SimpleNamespace

import numpy as np
from PIL import Image

from msgq.visionipc import VisionIpcClient
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.starpilot.galaxy.camera_snapshot import MAX_JPEG_BYTES
from openpilot.system.camerad.snapshot import extract_image


STREAMS = {"cabin": VisionStreamType.VISION_STREAM_CABIN, "wide": VisionStreamType.VISION_STREAM_WIDE_ROAD,
           "narrow": VisionStreamType.VISION_STREAM_NARROW_ROAD}
CABIN_SETTLE_NS = 10_000_000_000
# Road cameras start dark when the lease cold-starts camerad: the first fresh frame comes before auto exposure has
# ramped (sentry wide images 2026-10-22 22:10 and 23:48 were flat black, median 25 / p99 26). Take the frame once
# camerad reports its measured grey at its target for a few frames running, which is quick in daylight; in the dark
# the sensor tops out below target, so wait up to the 4 s the stock snapshot uses.
ROAD_SETTLE_NS = 4_000_000_000
SETTLED_TOLERANCE = 0.2  # |measured / target - 1|
SETTLED_FRAMES = 3       # camerad applies exposure 3 frames late; one frame on target can be a pass-through
CAMERA_STATES = {"wide": "wideRoadCameraState", "narrow": "roadCameraState"}
# camerad exposes for the driving model (grey target 0.1-0.4), not for people: stretch and lift road snapshots.
MAX_STRETCH = 4.0
TARGET_MEDIAN = 0.4
MAX_LIFT = 2.0  # shadow gain of the lift curve is 1 + lift; with MAX_STRETCH, no step gets more than 12x
# At night camerad runs the sensor at full gain and the lift multiplies that grain (sentry test 2026-10-23 00:33 CDT,
# lift 3.0, single frame: speckled). The car is parked, so average consecutive frames when the scene is dark: 8 frames
# (~0.4 s) cut sensor noise to ~1/3 at the cost of a slight smear on anything moving. Bright scenes keep one frame.
# Average the raw NV12 buffers (<1 ms each) and convert once: extract_image takes ~0.5 s per wide frame on the comma,
# and converting all 8 took the wide capture to 12 s (test 2026-10-23 00:50 CDT).
AVERAGE_FRAMES = 8
AVERAGE_BELOW_MEDIAN = 64  # Y-plane median, 0-255


def boot_ns():
  return time.clock_gettime_ns(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))


class ExposureWatch:
  """Has camerad's auto exposure reached its target? Falls back to the timer if camera states are unavailable."""
  def __init__(self, camera):
    from openpilot.cereal import messaging
    self.service = CAMERA_STATES[camera]
    self.messages = messaging.SubMaster([self.service])
    self.run = 0

  def settled(self):
    self.messages.update(0)
    if self.messages.updated[self.service]:
      state = self.messages[self.service]
      target, measured = state.targetGreyFraction, state.measuredGreyFraction
      on_target = target > 0 and measured > 0 and abs(measured / target - 1) <= SETTLED_TOLERANCE
      self.run = self.run + 1 if on_target else 0
    return self.run >= SETTLED_FRAMES


def _exposure_watch(camera):
  try:
    return ExposureWatch(camera)
  except Exception:
    return None


def brighten(rgb):
  """Levels + gamma for a road snapshot. Gain is capped so an unlit scene does not turn into amplified noise."""
  luma = rgb.mean(axis=2)
  lo, hi = np.percentile(luma, [1, 99.5])
  hi = max(hi, lo + 255 / MAX_STRETCH)
  levels = np.clip((np.arange(256, dtype=np.float32) - lo) / (hi - lo), 0, 1)
  median = max(float(levels[int(np.median(luma))]), 1e-3)
  # y = x(1 + k) / (1 + kx) puts the median at TARGET_MEDIAN without the unbounded shadow slope of a gamma curve
  lift = float(np.clip((TARGET_MEDIAN - median) / ((1 - TARGET_MEDIAN) * median), 0.0, MAX_LIFT))
  curve = levels * (1 + lift) / (1 + lift * levels)
  return np.round(255 * curve).astype(np.uint8)[rgb]


def capture(camera, *, client_factory=VisionIpcClient, clock=boot_ns, exposure_factory=_exposure_watch):
  started = clock()
  exposure = exposure_factory(camera) if camera != "cabin" else None
  client = client_factory("camerad", STREAMS[camera], True)
  while not client.connect(False):
    if clock() - started > 9_000_000_000:
      return None
    time.sleep(.1)
  connected = clock()
  first_frame = None
  total = count = dark = None
  while clock() - started < (22_000_000_000 if camera == "cabin" else 15_000_000_000):
    frame = client.recv(timeout_ms=100)
    now = clock()
    if camera == "cabin" and now - connected < CABIN_SETTLE_NS:
      continue
    if camera != "cabin" and frame is not None:
      first_frame = now if first_frame is None else first_frame
      settled = exposure is not None and exposure.settled()
      if not settled and now - first_frame < ROAD_SETTLE_NS:
        continue
    # Native camerad does not populate VIPC extra.valid; use capture freshness.
    if frame is None or not started < client.timestamp_sof <= client.timestamp_eof <= now:
      continue
    if now - client.timestamp_sof > 500_000_000:
      continue
    width, height, stride, uv_offset = frame.width, frame.height, frame.stride, frame.uv_offset
    uv_height = ((height // 2 + 15) // 16) * 16
    if (not 0 < width <= 4096 or not 0 < height <= 4096 or width % 2 or height % 2 or
        not width <= stride <= 8192 or stride % 2 or not stride * height <= uv_offset <= stride * 4096 or
        uv_offset % stride or len(frame.data) < uv_offset + stride * uv_height):
      return None
    if camera != "cabin":
      raw = np.frombuffer(frame.data, dtype=np.uint8, count=uv_offset + stride * uv_height)
      if total is None:
        total, count = raw.astype(np.uint16), 1
        dark = np.median(raw[:uv_offset].reshape(-1, stride)[:height:8, :width:8]) < AVERAGE_BELOW_MEDIAN
      else:
        total += raw
        count += 1
      if dark and count < AVERAGE_FRAMES:
        continue
      frame = SimpleNamespace(width=width, height=height, stride=stride, uv_offset=uv_offset,
                              data=((total + count // 2) // count).astype(np.uint8))
    image = Image.fromarray(extract_image(frame))
    image.thumbnail((1280, 720))
    if camera != "cabin":
      image = Image.fromarray(brighten(np.asarray(image)))
    output = BytesIO()
    image.save(output, format="JPEG", quality=85)
    body = output.getvalue()
    return body if len(body) <= MAX_JPEG_BYTES else None
  return None


if __name__ == '__main__':
  try:
    result = capture(sys.argv[1]) if len(sys.argv) == 2 and sys.argv[1] in STREAMS else None
    if result is not None:
      sys.stdout.buffer.write(result)
    else:
      sys.exit(1)
  except Exception:
    sys.exit(1)
