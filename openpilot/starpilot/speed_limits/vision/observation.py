"""Clock-aware projection of one typed vision event into the SLC domain."""

from __future__ import annotations

import math
import time

from openpilot.starpilot.speed_limits import acceptance as acc
from openpilot.starpilot.speed_limits import selection as sel

MAX_FRAME_AGE_NS = 250_000_000
# Bound sign-processing latency independently of raw camera admission.
MAX_RESULT_AGE_NS = 2_000_000_000
MAX_PAIR_SKEW_NS = 5_000_000
# How long one published sign result stays usable: receipt age, the producer's
# validUntil horizon, and the consumer's camera-frame age. Replay of one drive:
# publishes arrive every 1.08 s median (p90 1.56 s, p99 2.38 s, max 3.12 s) and
# lag their frame by ~0.5 s, so 1 s/2 s bounds went STALE between normal
# publishes and cleared the 3 s automatic-adoption pending. A sign that stops
# being published still clears within 3 s.
SIGN_HOLD_NS = 3_000_000_000
MODEL_ID = "82408b68c79c269296f0af942130c5383cace4ee06c78e2a4690e8488720116a:07c6696e530eb940d2757d5849b4bc0f1d785cda704e5296e18c0a94959f30a5"


def clock_pair_ns() -> tuple[int, int] | None:
  """Sample MONOTONIC around BOOTTIME; reject an interrupted sample."""
  before = time.monotonic_ns()
  boot_clock = getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC)
  boot = time.clock_gettime_ns(boot_clock)
  after = time.monotonic_ns()
  return ((before + after) // 2, boot) if 0 <= after - before <= MAX_PAIR_SKEW_NS else None


def vision_observation(event, *, receipt_mono_ns: int, now_mono_ns: int, now_boot_ns: int,
                       car_fingerprint: str, drive_session: str) -> acc.Observation:
  """An absent/invalid producer is unknown; a stale producer is never absence."""
  unknown = acc.Observation(acc.ObservationKind.UNKNOWN)
  stale = acc.Observation(acc.ObservationKind.STALE)
  if not car_fingerprint or not drive_session or receipt_mono_ns <= 0:
    return unknown
  if not 0 <= now_mono_ns - receipt_mono_ns <= SIGN_HOLD_NS:
    return stale
  if str(event.modelId) != MODEL_ID or str(event.stream) not in ("road", "wideRoad"):
    return unknown
  status = str(event.status)
  if status == "unavailable" or status == "unknown":
    return unknown
  if status == "stale":
    return stale
  if status != "valid":
    return unknown
  eof_boot_ns = int(event.cameraFrameEofBootTime)
  observed_ns = int(event.observedMonoTime)
  expiry_ns = int(event.validUntilMonoTime)
  if (eof_boot_ns <= 0 or not 0 <= now_boot_ns - eof_boot_ns <= SIGN_HOLD_NS or
      observed_ns <= 0 or observed_ns > now_mono_ns or
      expiry_ns <= observed_ns or now_mono_ns > expiry_ns):
    return stale
  speed, confidence = float(event.speedMps), float(event.confidence)
  if (not math.isfinite(speed) or not 1.0 <= speed <= 55.0 or
      not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0 or
      int(event.supportCount) <= 0 or int(event.episode) <= 0 or not str(event.producerSessionId)):
    return unknown
  identity = acc.ObservationIdentity(acc.IdentityKind.PRODUCER_EPISODE,
                                     value=f"{drive_session}:{car_fingerprint}:{event.producerSessionId}:{int(event.episode)}")
  return acc.Observation(acc.ObservationKind.VALID,
                         acc.Candidate(sel.Source.VISION.value, identity, speed))
