"""Bosch-A XCP assist snapshot as a second sighting source for the V-ASM radar pass hold (radar_pass_hold.py).

Reads /dev/shm/xcp_radar_assist.json (writer: starpilot/car/honda/xcp_radar_logger.py `_write_ui_snapshot`; reader of the
same file for lead matching: starpilot/car/honda/radar.py `_read_xcp_snapshot`/`_xcp_target_qualifies`). Schema used:
  {"mono_ns": int, "targets": [{"stage": "measured", "ghost": bool, "flags": int, "age": int, "dRel_m": float,
   "yRel_m": float (LEFT-positive), "vRel_mps": float (negative = we are closing on it), "kin_age_ms": float,
   "lat_valid": bool, "rtid": int, ...}, ...]}
`mono_ns` is time.monotonic_ns() in the writer (card.py passes the same clock), which is also the spot-monitor runtime's
mono_clock, so no boot_offset_ns correction is applied (radar.py applies it only when it runs on logMonoTime).

Fail-closed: a missing, unreadable, malformed or stale file yields no sightings, never an exception, and this module can
only ever feed sightings to a hold that itself only ADDS "occupied". Static unit tests only; not road-validated.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from openpilot.starpilot.spot_monitor.radar_pass_hold import PASS_MIN_CLOSING, AdjacentLead

XCP_ASSIST_PATH = Path("/dev/shm/xcp_radar_assist.json")

# Duplicated from starpilot/car/honda/radar.py (XCP_ASSIST_*): importing radar.py pulls capnp and the radard stack.
SNAPSHOT_MAX_AGE_NS = 200_000_000   # radar.py XCP_ASSIST_MAX_SNAPSHOT_AGE_NS
SNAPSHOT_MAX_FUTURE_SKEW_NS = 60_000_000  # radar.py XCP_ASSIST_MAX_FUTURE_SKEW_NS
MAX_KIN_AGE_MS = 200.0              # radar.py XCP_ASSIST_MAX_KIN_AGE_MS
MIN_AGE = 5                         # radar.py XCP_ASSIST_MIN_AGE
BAD_FLAGS = 0x1020                  # radar.py: (flags & 0x1020) != 0 rejects

# Lateral bands, y is LEFT-positive (verified on routes). Anything else is in-lane, two lanes over, or noise.
SIDE_Y_MIN_M = 1.8
SIDE_Y_MAX_M = 5.4

# 15 m camera-bank window + ~7 m median XCP-camera range excess, routes 2026-11 replay study; add-only so widening
# costs nuisance holds, not safety. Raw XCP range: no 434/128 offset subtracted (no constant offset in the study).
XCP_MAX_D_M = 22.0

# Only vehicles moving our way: ground speed vEgo + vRel >= 1.5 m/s. Replay 2026-11-18 (Long Sim, 9 XCP routes, 8422 s):
# unfiltered, XCP added 660/646 s (left/right) of hold, 65-73% on static targets plus oncoming; with this gate 141/176 s,
# XCP-only holds with nothing nearby 72/96 -> 1/5 episodes, 23 of 63 camera-missed passes kept (the 40 lost are oncoming).
# 1.0 and 3.0 m/s scored similarly. No vEgo -> no sightings (fail-closed: the hold only adds).
MIN_GROUND_SPEED_MPS = 1.5


def _qualifies(t: Any) -> tuple[float, float, float, int] | None:
  """radar.py's assist gates minus its consistency-to-op-track checks. Returns (d, y, v_rel, rtid) or None."""
  try:
    if t.get("stage") != "measured" or bool(t.get("ghost", False)):
      return None
    if "lat_valid" in t and not bool(t.get("lat_valid")):
      return None
    if "lat_updated_ns" in t and int(t.get("lat_updated_ns") or 0) <= 0:
      return None
    if t.get("yRel_m") is None or t.get("kin_age_ms") is None:  # absent kin_age: not qualified
      return None
    kin_age_ms, flags, age = float(t["kin_age_ms"]), int(t.get("flags", 0)), int(t.get("age", 0))
    d, y, v = float(t["dRel_m"]), float(t["yRel_m"]), float(t["vRel_mps"])
    rtid = int(t.get("rtid", -1))
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None
  if not all(math.isfinite(x) for x in (kin_age_ms, d, y, v)):
    return None
  if kin_age_ms > MAX_KIN_AGE_MS or (flags & BAD_FLAGS) != 0 or age < MIN_AGE:
    return None
  return d, y, v, rtid


class XcpSideSightings:
  def __init__(self, path: Path | str | None = XCP_ASSIST_PATH):
    self.path = Path(path) if path is not None else None

  def _snapshot(self, now_ns: int) -> dict | None:
    if self.path is None:
      return None
    try:
      data = json.loads(self.path.read_bytes())
      if not isinstance(data, dict):
        return None
      age_ns = now_ns - int(data["mono_ns"])
    except Exception:  # missing, unreadable, malformed, no timestamp
      return None
    if not (-SNAPSHOT_MAX_FUTURE_SKEW_NS <= age_ns <= SNAPSHOT_MAX_AGE_NS):
      return None
    return data

  def read(self, now_ns: int, v_ego: float | None = None) -> dict[str, AdjacentLead | None]:
    """One file read. Car-frame sides: left = positive y. Per side the nearest closing target in the window, else the
    nearest non-closing one (present, so a pull-away is seen as such), else None."""
    out: dict[str, AdjacentLead | None] = {"left": None, "right": None}
    try:
      v_ego_f = float(v_ego) if v_ego is not None else math.nan
    except (TypeError, ValueError):
      v_ego_f = math.nan
    if not math.isfinite(v_ego_f):
      return out
    snap = self._snapshot(now_ns)
    if snap is None:
      return out
    try:
      targets = list(snap.get("targets") or ())
    except TypeError:
      return out
    best: dict[str, tuple[tuple[bool, float], AdjacentLead]] = {}
    for t in targets:
      q = _qualifies(t)
      if q is None:
        continue
      d, y, v, rtid = q
      if not 0.0 <= d <= XCP_MAX_D_M or v_ego_f + v < MIN_GROUND_SPEED_MPS:
        continue
      if SIDE_Y_MIN_M <= y <= SIDE_Y_MAX_M:
        side = "left"
      elif -SIDE_Y_MAX_M <= y <= -SIDE_Y_MIN_M:
        side = "right"
      else:
        continue
      key = (-v < PASS_MIN_CLOSING, d)  # closing targets sort first, then nearest
      if side not in best or key < best[side][0]:
        best[side] = (key, AdjacentLead(True, d, v, rtid))
    for side, (_, lead) in best.items():
      out[side] = lead
    return out
