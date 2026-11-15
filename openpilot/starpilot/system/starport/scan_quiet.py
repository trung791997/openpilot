"""Keep Wi-Fi scans off the shared radio while StarPort streams Android Auto to the car.

The comma has one Qualcomm radio that is not DBS: aa0 (the car's projection link, 5 GHz) and wlan0 (phone hotspot or home
Wi-Fi) share it, and a scan takes it off aa0's channel. 2026-11-05 16:02 CST: wlan0 lost home Wi-Fi (a wifi.hidden=yes
profile), NetworkManager went to active scanning for hidden networks at 16:02:22, and 16 s later StarPort's send to the
head unit timed out and aa0 dropped.

starportd's wlan0 watchdog (wifi_rejoin.HotspotRejoin) sees the streaming loop's heartbeat and holds a lease here: a tiny
file in tmpfs with a CLOCK_MONOTONIC deadline, renewed every couple of seconds. Any process (the UI's Wi-Fi manager too)
skips its own scans while the lease is fresh. Nothing has to clear it: a crashed or hung starportd stops renewing it and
scans come back within STALE_AFTER s. tmpfs and the monotonic clock both reset at boot, so a lease never outlives one.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

LEASE_PATH = Path("/dev/shm/starport_scan_quiet.json")
STALE_AFTER = 10.  # s without a streaming heartbeat (or lease renewal) before scans come back
STREAMING_STATES = ("streaming", "suspended")  # supervisor states with a live car link on aa0


def fresh(beat: float | None, now: float) -> bool:
  """True when a streaming heartbeat at monotonic ``beat`` is recent enough to keep the radio quiet."""
  return beat is not None and 0. <= now - beat < STALE_AFTER


def hold(until: float, path: Path = LEASE_PATH) -> bool:
  """Hold (or renew) the lease until monotonic ``until``. False when it could not be written."""
  tmp = path.with_name(f".{path.name}.{os.getpid()}")
  try:
    tmp.write_text(json.dumps({"until": round(until, 3), "pid": os.getpid()}))
    os.replace(tmp, path)
    return True
  except OSError:
    try:
      tmp.unlink(missing_ok=True)
    except OSError:
      pass
    return False


def drop(path: Path = LEASE_PATH) -> None:
  try:
    path.unlink(missing_ok=True)
  except OSError:
    pass


def quiet(now: float | None = None, path: Path = LEASE_PATH) -> bool:
  """True while StarPort holds a fresh lease: callers should not ask any Wi-Fi interface to scan.

  Anything unreadable, expired, or implausibly far in the future reads as False, so a fault here never blocks scans for long.
  """
  now = time.monotonic() if now is None else now
  try:
    until = float(json.loads(path.read_text())["until"])
  except (OSError, ValueError, KeyError, TypeError):
    return False
  return now < until <= now + 2 * STALE_AFTER
