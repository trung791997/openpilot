from datetime import UTC
from zoneinfo import ZoneInfo

import pytest

from openpilot.starpilot import local_timezone


class Clock:
  def __init__(self):
    self.now = 100.0

  def __call__(self):
    return self.now


@pytest.fixture
def zone_store(monkeypatch, tmp_path):
  clock = Clock()
  monkeypatch.setattr(local_timezone, "_root", lambda: tmp_path / "state")
  monkeypatch.setattr(local_timezone, "_monotonic", clock)
  monkeypatch.setattr(local_timezone, "_cache", None)
  return tmp_path / "state" / local_timezone.FILE, clock


def test_store_valid_invalid_and_unchanged(zone_store):
  path, _ = zone_store
  assert local_timezone.store_timezone("America/Los_Angeles")
  assert path.read_text() == "America/Los_Angeles"
  mtime = path.stat().st_mtime_ns
  assert local_timezone.store_timezone("America/Los_Angeles")
  assert path.stat().st_mtime_ns == mtime
  for bad in ("", "Not/AZone", "../etc/passwd", "x" * 129, None, 5, b"UTC", "UTC\n\x00"):
    assert local_timezone.store_timezone(bad) is False  # ty: ignore[invalid-argument-type]
  assert path.read_text() == "America/Los_Angeles"


def test_store_unwritable_returns_false(monkeypatch, tmp_path):
  blocker = tmp_path / "file"
  blocker.write_text("x")
  monkeypatch.setattr(local_timezone, "_root", lambda: blocker / "sub")
  assert local_timezone.store_timezone("UTC") is False


def test_fallback_to_utc_when_missing_or_invalid(zone_store):
  path, clock = zone_store
  assert local_timezone.local_zone() is UTC
  path.parent.mkdir(parents=True)
  path.write_text("Garbage/Zone")
  clock.now += 11
  assert local_timezone.local_zone() is UTC


def test_cache_refreshes_every_ten_seconds(zone_store):
  path, clock = zone_store
  assert local_timezone.local_zone() is UTC
  path.parent.mkdir(parents=True)
  path.write_text("Asia/Tokyo")
  clock.now += 9.9
  assert local_timezone.local_zone() is UTC
  clock.now += 0.2
  assert local_timezone.local_zone() == ZoneInfo("Asia/Tokyo")
  path.write_text("Europe/Paris")
  clock.now += 5
  assert local_timezone.local_zone() == ZoneInfo("Asia/Tokyo")
  clock.now += 6
  assert local_timezone.local_zone() == ZoneInfo("Europe/Paris")


def test_store_takes_effect_immediately_and_local_now_uses_zone(zone_store):
  assert local_timezone.local_now().utcoffset().total_seconds() == 0
  assert local_timezone.store_timezone("America/Los_Angeles")
  assert local_timezone.local_zone() == ZoneInfo("America/Los_Angeles")
  assert local_timezone.local_now().tzinfo == ZoneInfo("America/Los_Angeles")
