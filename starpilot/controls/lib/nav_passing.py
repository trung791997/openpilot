"""D-117: jammed-exit notice.

Prompt only. Nothing here steers: the driver's blinker starts the existing lane change, which keeps its own
blind-spot block. Starting values, not tuned; no replay or road evidence yet.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SHOULDER_EDGE_GAP = 1.5       # m: road edge closer than this beyond the outer line = a shoulder, not a lane (D-114 Open)
EXIT_JAM_WARN_DIST = 3000.0   # D-117: suggest moving over early for a jammed exit from this far out


@dataclass
class PassingInputs:
  nav_state: dict           # NavInstructionState
  lane_width_left: float
  lane_width_right: float
  lane_detection_width: float
  left_blinker: bool
  right_blinker: bool
  edge_gap_left: float = 99.0   # m, road edge beyond the outer lane line at 20 m (edge_gap()); 99 = unknown
  edge_gap_right: float = 99.0
  exit_jam_warn: bool = False  # D-117 NavJamExitWarn toggle


def lane_beside(width: float, detection_width: float, edge_gap: float = 99.0) -> bool:
  """A drivable lane on that side.

  Width alone counts a paved shoulder as a lane: on I-35 (00000324, 00000321, 2026-10-12) the model draws the outer
  line on the road edge in the edge lane, so the "lane" beyond the yellow or right white line still measures 3+ m. The
  edge then sits within ~1 m of that line (shoulder p95 0.86 m), while a real lane leaves the edge 2.2+ m beyond the
  next lane's outer line (p1 2.17 m). /routes/an2/shsep.py.
  """
  return width >= detection_width and edge_gap >= SHOULDER_EDGE_GAP


def _left_lane(i: PassingInputs) -> bool:
  return lane_beside(i.lane_width_left, i.lane_detection_width, i.edge_gap_left)


def _right_lane(i: PassingInputs) -> bool:
  return lane_beside(i.lane_width_right, i.lane_detection_width, i.edge_gap_right)


def edge_gap(line_x, line_y, edge_x, edge_y, x: float = 20.0) -> float:
  """How far the road edge lies beyond an outer lane line, lateral metres at x metres ahead."""
  return abs(float(np.interp(x, edge_x, edge_y))) - abs(float(np.interp(x, line_x, line_y)))


def _float(nav_state: dict, key: str) -> float:
  try:
    return float(nav_state.get(key, -1.0))
  except (TypeError, ValueError):
    return -1.0


class NavPassingAdvisor:
  @staticmethod
  def _exit_jam(i: PassingInputs) -> str:
    """D-117: the side to move to early when the route's exit is jammed and a lane is still between us and it."""
    side = i.nav_state.get("exitJamSide", "")
    distance = _float(i.nav_state, "exitJamDistance")
    if side not in ("left", "right") or not 0.0 < distance <= EXIT_JAM_WARN_DIST:
      return ""
    lane_toward = _left_lane(i) if side == "left" else _right_lane(i)
    blinker = i.left_blinker if side == "left" else i.right_blinker
    return side if lane_toward and not blinker else ""

  def update(self, i: PassingInputs, now: float) -> dict:
    nav_ok = bool(i.nav_state) and bool(i.nav_state.get("valid", False))
    # Route information, not a lane change suggestion: no V-ASM needed.
    exit_jam = self._exit_jam(i) if i.exit_jam_warn and nav_ok else ""
    if exit_jam:
      return {"armed": True, "side": exit_jam, "phase": "exitjam", "ts": now,
              "distance": _float(i.nav_state, "exitJamDistance")}
    return {"armed": False, "side": "", "phase": "", "ts": now}
