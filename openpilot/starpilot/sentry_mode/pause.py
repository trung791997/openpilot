"""A saved wall-clock resume time for Sentry, independent of the enabled preference.

SentryPausedUntil holds an ASCII integer unix epoch second or is absent. Pausing never
rewrites the saved `enabled` preference, so resuming (by expiry or "Resume now") returns
to exactly the prior setting.
"""

import time
from typing import Any

KEY = "SentryPausedUntil"
# Owner request: Pause is a short break with presets up to 8 h or a picked time, not a
# silent indefinite disable. A saved value further out than this is treated as invalid.
MAX_SECONDS = 30 * 24 * 60 * 60
# 2024-01-01T00:00:00Z. A wall clock reporting a time before this has not been set yet
# (e.g. booted without NTP); Sentry fails active rather than trusting a stale pause.
MIN_VALID_NOW = 1704067200


def read_pause(params: Any) -> int | None:
  """Return the saved pause-until epoch, or None if absent or unparseable (clearing the latter)."""
  raw = params.get(KEY)
  if raw is None:
    return None
  try:
    text = raw.decode() if isinstance(raw, bytes) else raw
    if not text or not text.lstrip("-").isdigit():
      raise ValueError("not an integer")
    return int(text)
  except (ValueError, UnicodeDecodeError, AttributeError):
    clear_pause(params)
    return None


def set_pause_until(params: Any, epoch: int, *, now: float | None = None) -> None:
  """Save a pause end time. Rejects the past and anything more than 30 days out."""
  if isinstance(epoch, bool) or not isinstance(epoch, int):
    raise ValueError("pause end must be an integer epoch")
  moment = now if now is not None else time.time()
  if epoch <= moment:
    raise ValueError("pause end must be in the future")
  if epoch - moment > MAX_SECONDS:
    raise ValueError("pause end is more than 30 days away")
  params.put(KEY, str(epoch), block=True)


def clear_pause(params: Any) -> None:
  params.remove(KEY)


def is_paused(params: Any, now: float) -> bool:
  """Fail-active: an invalid clock, or an invalid/expired/too-distant saved value, means not paused."""
  if now < MIN_VALID_NOW:
    return False
  until = read_pause(params)
  if until is None or until - now > MAX_SECONDS:
    return False
  if now >= until:
    clear_pause(params)
    return False
  return True
