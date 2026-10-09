#!/usr/bin/env python3
"""Haptic navigation steering wheel torque pulse assistance."""
from __future__ import annotations

import json
import numpy as np

from cereal import log
from openpilot.common.params import Params

TURN_NUDGE_TORQUE = 0.8
EXIT_NUDGE_TORQUE = 0.6
TURN_PULSE_FRAMES = 50
EXIT_PULSE_FRAMES = 75
PULSE_TRIGGER_DISTANCE_M = 200.0

# Hard-off, as in IQ.Pilot (IQP_NAV_TORQUE_INFLUENCE_ENABLED). Nothing calls this class yet either.
# IQ.Pilot adds the pulse in latcontrol_torque before the controller's final negation, so the signs
# below are only right at that point; added after the negation they would steer the wrong way.
NAV_TORQUE_INFLUENCE_ENABLED = False


class NavTorquePulse:
  def __init__(self, steer_max: float = 1.0):
    self.steer_max = float(steer_max)
    self.params_memory = Params(memory=True)
    self._nav_key = ""
    self._nav_pulse_sign = 0.0
    self._nav_pulse_frames = 0
    self._cached_state: dict[str, object] = {}
    self._raw_state: object = None

  def _update_nav_state(self) -> dict[str, object]:
    raw = self.params_memory.get("NavInstructionState") or {}
    if raw == self._raw_state:
      return self._cached_state
    self._raw_state = raw
    if isinstance(raw, dict):
      self._cached_state = raw
    elif isinstance(raw, (str, bytes)):
      try:
        parsed = json.loads(raw)
        self._cached_state = parsed if isinstance(parsed, dict) else {}
      except Exception:
        self._cached_state = {}
    else:
      self._cached_state = {}
    return self._cached_state

  def _lookup_nav_pulse(self) -> tuple[str, float, int]:
    if not NAV_TORQUE_INFLUENCE_ENABLED:
      return "", 0.0, 0

    nav_state = self._update_nav_state()
    if not bool(nav_state.get("valid", False)):
      return "", 0.0, 0

    try:
      distance = float(nav_state.get("maneuverDistance", 9999.0))
    except (TypeError, ValueError):
      return "", 0.0, 0

    if distance > PULSE_TRIGGER_DISTANCE_M or distance <= 0.0:
      return "", 0.0, 0

    maneuver_type = str(nav_state.get("maneuverType", "")).lower()
    modifier = str(nav_state.get("maneuverModifier", ""))

    # left nudges negative, otherwise positive (pre-negation, see NAV_TORQUE_INFLUENCE_ENABLED)
    def turn_pulse(direction_str: str) -> tuple[str, float, int]:
      sign = -TURN_NUDGE_TORQUE if "left" in direction_str.lower() else TURN_NUDGE_TORQUE
      return f"turn:{direction_str}", sign, TURN_PULSE_FRAMES

    def exit_pulse(direction_str: str) -> tuple[str, float, int]:
      sign = -EXIT_NUDGE_TORQUE if "left" in direction_str.lower() else EXIT_NUDGE_TORQUE
      return f"exit:{direction_str}", sign, EXIT_PULSE_FRAMES

    if maneuver_type in ("off ramp", "fork", "exit") or "exit" in modifier.lower():
      return exit_pulse(modifier)

    if modifier in ("left", "sharpLeft", "right", "sharpRight"):
      return turn_pulse(modifier)

    return "", 0.0, 0

  def nudge_output_torque(self, active: bool, carstate, output_torque: float) -> float:
    if not NAV_TORQUE_INFLUENCE_ENABLED:
      self._nav_pulse_frames = 0
      self._nav_key = ""
      return output_torque

    nav_key, pulse_sign, pulse_frames = self._lookup_nav_pulse()

    if not active or getattr(carstate, "steeringPressed", False):
      self._nav_pulse_frames = 0
      if not nav_key:
        self._nav_key = ""
      return output_torque

    if nav_key and nav_key != self._nav_key:
      self._nav_key = nav_key
      self._nav_pulse_sign = pulse_sign
      self._nav_pulse_frames = pulse_frames
    elif not nav_key and self._nav_pulse_frames == 0:
      self._nav_key = ""

    if self._nav_pulse_frames > 0:
      self._nav_pulse_frames -= 1
      output_torque = float(np.clip(output_torque + self._nav_pulse_sign, -self.steer_max, self.steer_max))

    return output_torque
