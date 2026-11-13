"""The phone's IANA timezone, saved by Galaxy, so the UTC-clocked device can show local wall-clock times."""

from collections.abc import Callable
from datetime import UTC, datetime, tzinfo
import os
from pathlib import Path
import tempfile
import time
from zoneinfo import ZoneInfo

from openpilot.starpilot.storage import starpilot_storage_root


FILE = "local-timezone"
MAX_NAME = 128
REFRESH_SECONDS = 10.0

# Test seams: production code never changes these.
_root: Callable[[], Path] = starpilot_storage_root
_monotonic: Callable[[], float] = time.monotonic
_cache: tuple[float, str, tzinfo] | None = None


def _valid(name) -> ZoneInfo | None:
  if type(name) is not str or not 1 <= len(name) <= MAX_NAME:
    return None
  try:
    return ZoneInfo(name)
  except (KeyError, ValueError, OSError):
    return None


def _read() -> str:
  try:
    return (_root() / FILE).read_text(encoding="utf-8")[:MAX_NAME + 1].strip()
  except (OSError, ValueError):
    return ""


def store_timezone(name: str) -> bool:
  """Save a valid IANA zone; True when the stored value is now `name`. Never raises."""
  global _cache
  if _valid(name) is None:
    return False
  try:
    if _read() == name:
      return True
    root = _root()
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".local-timezone-", dir=root)
    try:
      with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(name)
        stream.flush()
        os.fsync(stream.fileno())
      os.replace(temporary, root / FILE)
    finally:
      if os.path.exists(temporary):
        os.unlink(temporary)
  except (OSError, ValueError):
    return False
  _cache = None
  return True


def local_zone() -> tzinfo:
  """The saved zone, or UTC; storage is re-read at most every REFRESH_SECONDS."""
  global _cache
  now = _monotonic()
  if _cache is not None and 0 <= now - _cache[0] < REFRESH_SECONDS:
    return _cache[2]
  name = _read()
  zone = _valid(name) or UTC
  _cache = (now, name, zone)
  return zone


def local_now() -> datetime:
  return datetime.now(local_zone())
