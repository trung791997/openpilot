"""CPU-only frame reader. Does not start or stop camerad or change Params."""

from io import BytesIO
import sys
import time

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
# ramped (sentry wide images 2026-10-22 22:10 and 23:48 were flat black, median 25 / p99 26). The stock snapshot waits
# 4 s of frames for the same reason.
ROAD_SETTLE_NS = 4_000_000_000
# camerad exposes for the driving model (grey target 0.1-0.4), not for people: stretch and lift road snapshots.
MAX_STRETCH = 4.0
TARGET_MEDIAN = 0.4
MAX_LIFT = 3.0  # shadow gain of the lift curve is 1 + lift; with MAX_STRETCH, no step gets more than 16x


def boot_ns():
  return time.clock_gettime_ns(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))


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


def capture(camera, *, client_factory=VisionIpcClient, clock=boot_ns):
  started = clock()
  client = client_factory("camerad", STREAMS[camera], True)
  while not client.connect(False):
    if clock() - started > 9_000_000_000:
      return None
    time.sleep(.1)
  connected = clock()
  first_frame = None
  while clock() - started < (22_000_000_000 if camera == "cabin" else 15_000_000_000):
    frame = client.recv(timeout_ms=100)
    now = clock()
    if camera == "cabin" and now - connected < CABIN_SETTLE_NS:
      continue
    if camera != "cabin" and frame is not None:
      first_frame = now if first_frame is None else first_frame
      if now - first_frame < ROAD_SETTLE_NS:
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
