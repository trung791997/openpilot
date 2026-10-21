#!/usr/bin/env python3
import time

from openpilot.common.constants import CV

from openpilot.starpilot.common.experimental_state import (
  CCStatus,
  is_manual_cc_status,
  restore_persisted_cc_state,
)


class DetectorToggles:
  """Fixed detector configuration used by Conditional Chill."""

  __slots__ = ()

  conditional_curves = True
  conditional_curves_lead = True
  conditional_lead = True
  conditional_slower_lead = True
  conditional_stopped_lead = True


DETECTOR_TOGGLES = DetectorToggles()


class ConditionalChillMode:
  CCM_STOP_MODEL_TIME = 7.0
  CHILL_SPEED_ENTRY_CONFIRM_TIME = 0.35
  CHILL_LEAD_ENTRY_CONFIRM_TIME = 1.0
  CHILL_LAUNCH_ENTRY_CONFIRM_TIME = 0.0
  CHILL_EXIT_BUFFER_TIME = 0.35
  CHILL_MIN_DWELL_TIME = 1.2

  # Launch assist (2026-10-21, Peter: "switch to chill from stop regardless of launching behind a lead,
  # at a light, at a stop sign"). The old law needed a departing lead and a clean stop scene, so it
  # almost never fired: a stopped lead tripped the slow-lead veto, and the lead-departure thresholds
  # were re-checked every cycle. Now: arm while stopped, switch to Chill only once the car is actually
  # rolling (experimental or the driver decides when to go, so Chill never pulls away from a red
  # light), and hold Chill to CHILL_LAUNCH_EXIT_SPEED. Stopping again always hands back to experimental.
  #
  # Job log check (STATUS 261, 3a7/3a3): queue creeps stopped again ~1.2 s / 0.6 m after 0.5 m/s, so a
  # 0.5 m/s start flipped modes on every creep; CHILL_LAUNCH_START_SPEED is 1.5 so creeps stay in
  # experimental. Pulling away on green, redLight came back on at 5.7 m and 7.1 m, so every stop flag
  # is ignored for the first CHILL_LAUNCH_STOP_FLAG_DISTANCE (the intersection just left). After that a
  # stop flag hands back to experimental. 20 m below 15 mph is at least 3 s in Chill, so no separate
  # minimum hold is needed.
  #
  # Job 311 jam replay (STATUS 261): stopping behind a queued lead in Chill never braked harder than
  # experimental but stopped ~1.5 m closer (3.2-3.8 m vs 4.9-5.5 m, min 2.46 m). Peter: hand back to
  # experimental for the stop. Past CHILL_LAUNCH_STOP_FLAG_DISTANCE, a lead that is stopped (slow-lead
  # detector, or under CHILL_LAUNCH_LEAD_STOPPED_SPEED) or slowing to a stop (under
  # CHILL_LAUNCH_LEAD_STOPPING_SPEED and decelerating) ends the launch.
  CHILL_LAUNCH_EXIT_SPEED = 15 * CV.MPH_TO_MS
  CHILL_LAUNCH_ARM_SPEED = 0.3
  CHILL_LAUNCH_START_SPEED = 1.5
  CHILL_LAUNCH_STOP_FLAG_DISTANCE = 20.0
  CHILL_LAUNCH_LEAD_STOPPED_SPEED = 1.0
  CHILL_LAUNCH_LEAD_STOPPING_SPEED = 2.0
  CHILL_LAUNCH_LEAD_STOPPING_DECEL = -0.3

  STABLE_LEAD_MIN_MODEL_PROB = 0.9
  STABLE_LEAD_MAX_BRAKE = 0.2
  STABLE_LEAD_MIN_SPEED = 1.5
  STABLE_LEAD_MAX_DISTANCE = 90.0
  STABLE_LEAD_MAX_DISTANCE_TIME = 4.5
  STABLE_LEAD_MAX_CLOSING_SPEED = 1.25
  STABLE_LEAD_MAX_CLOSING_RATIO = 0.05

  ADJACENT_LEAD_VETO_MIN_SPEED = 1.0
  ADJACENT_LEAD_VETO_MAX_DISTANCE = 65.0
  ADJACENT_LEAD_VETO_MAX_DISTANCE_TIME = 3.5
  ADJACENT_LEAD_VETO_MAX_LATERAL_OFFSET = 5.5

  LOW_SPEED_STOP_SCENE_MAX_SPEED = 18 * CV.MPH_TO_MS

  def __init__(self, StarPilotPlanner, detector):
    self.starpilot_planner = StarPilotPlanner
    self.detector = detector
    self.params = self.starpilot_planner.params
    self.params_memory = self.starpilot_planner.params_memory

    self.experimental_mode = True
    self.status_value = CCStatus["OFF"]
    self._active_auto_status = CCStatus["OFF"]
    self._candidate_since = 0.0
    self._soft_exit_since = 0.0
    self._chill_hold_until = 0.0
    self._prev_cc_status = None
    self._launch_active = False
    self._launch_forced_exit = False
    self._launch_armed = False
    self._launch_distance = 0.0
    self._launch_last_time = 0.0

  def update(self, v_ego, v_cruise, sm, starpilot_toggles):
    now = time.monotonic()
    safe_mode = self.params.get_bool("SafeMode")

    self.status_value = CCStatus["OFF"] if safe_mode else restore_persisted_cc_state(self.params, self.params_memory)

    if is_manual_cc_status(self.status_value):
      self._reset_timers()
      self._launch_armed = False
      self.experimental_mode = self.status_value == CCStatus["USER_EXPERIMENTAL"]
      self._write_status(self.status_value)
      return

    self._refresh_detector(v_ego, sm)
    auto_status, launch_candidate = self._get_chill_status(v_ego, v_cruise, sm, starpilot_toggles)

    if safe_mode or self._has_hard_veto(v_ego, sm, allow_launch=launch_candidate):
      self._reset_timers()
      self.experimental_mode = False if safe_mode else True
      self.status_value = CCStatus["OFF"]
      self._write_status(CCStatus["OFF"])
      return

    chill_candidate = auto_status != CCStatus["OFF"]
    entry_confirm_time = self._get_entry_confirm_time(auto_status, launch_candidate)

    if chill_candidate:
      if self._candidate_since == 0.0:
        self._candidate_since = now
      self._soft_exit_since = 0.0

      if not self.experimental_mode or (now - self._candidate_since) >= entry_confirm_time:
        self.experimental_mode = False
        self._active_auto_status = auto_status
        self._chill_hold_until = max(self._chill_hold_until, now + self.CHILL_MIN_DWELL_TIME)
      self.status_value = self._active_auto_status if not self.experimental_mode else CCStatus["OFF"]
    else:
      self._candidate_since = 0.0
      if not self.experimental_mode:
        if self._launch_forced_exit:
          self.experimental_mode = True
          self._active_auto_status = CCStatus["OFF"]
          self.status_value = CCStatus["OFF"]
          self._soft_exit_since = 0.0
          self._chill_hold_until = 0.0
          self._launch_forced_exit = False
          self._write_status(CCStatus["OFF"])
          return

        if self._soft_exit_since == 0.0:
          self._soft_exit_since = now

        hold_active = now < self._chill_hold_until
        exit_buffer_active = (now - self._soft_exit_since) < self.CHILL_EXIT_BUFFER_TIME
        if hold_active or exit_buffer_active:
          self.status_value = self._active_auto_status
        else:
          self.experimental_mode = True
          self._active_auto_status = CCStatus["OFF"]
          self.status_value = CCStatus["OFF"]
      else:
        self._soft_exit_since = 0.0
        self.status_value = CCStatus["OFF"]

    self._write_status(self.status_value if not self.experimental_mode else CCStatus["OFF"])

  def deactivate(self):
    self._reset_timers()
    self._launch_armed = False
    self.experimental_mode = True
    self._prev_cc_status = None

  def _reset_timers(self):
    self._active_auto_status = CCStatus["OFF"]
    self._candidate_since = 0.0
    self._soft_exit_since = 0.0
    self._chill_hold_until = 0.0
    self._launch_active = False
    self._launch_forced_exit = False

  def _refresh_detector(self, v_ego, sm):
    self.detector.curve_detection(v_ego, DETECTOR_TOGGLES)
    self.detector.slow_lead(DETECTOR_TOGGLES, v_ego)
    self.detector.stop_sign_and_light(v_ego, sm, self.CCM_STOP_MODEL_TIME)

  def _has_hard_veto(self, v_ego, sm, allow_launch=False):
    if sm["carState"].standstill and not allow_launch:
      return True

    if sm["carState"].leftBlinker or sm["carState"].rightBlinker:
      return True

    if sm["starpilotCarState"].trafficModeEnabled:
      return True

    if self.starpilot_planner.starpilot_vcruise.slc.experimental_mode:
      return True

    if self.detector.curve_detected:
      return True

    # During a launch the stopped/slow lead, stop light/sign and adjacent-lane cars are the scene the
    # car just pulled away from; _launch_exit_required decides which stop flags are fresh.
    if allow_launch:
      return False

    if self.detector.slow_lead_detected or self.detector.stop_light_detected:
      return True

    if self.starpilot_planner.starpilot_vcruise.stop_sign_confirmed or self.starpilot_planner.starpilot_vcruise.forcing_stop:
      return True

    if self._adjacent_lead_ambiguous(sm, v_ego):
      return True

    return self._low_speed_stop_scene(v_ego) and not allow_launch

  def _low_speed_stop_scene(self, v_ego):
    if v_ego >= self.LOW_SPEED_STOP_SCENE_MAX_SPEED:
      return False

    if self.starpilot_planner.raw_model_stopped or self.starpilot_planner.model_stopped or self.detector.stop_light_model_detected:
      return True

    lead = self.starpilot_planner.lead_one
    if not getattr(lead, "status", False):
      return False

    lead_distance = float(getattr(lead, "dRel", float("inf")))
    lead_speed = float(getattr(lead, "vLead", float("inf")))
    lead_distance_limit = max(40.0, v_ego * self.STABLE_LEAD_MAX_DISTANCE_TIME)
    return lead_distance < lead_distance_limit and lead_speed < max(6.0, v_ego + 0.5)

  def _get_chill_status(self, v_ego, v_cruise, sm, starpilot_toggles):
    if getattr(starpilot_toggles, "conditional_chill_launch_assist", False):
      launch_status = self._get_launch_status(v_ego, sm)
      if launch_status != CCStatus["OFF"]:
        return launch_status, True

    lead = self.starpilot_planner.lead_one
    lead_status = bool(getattr(lead, "status", False))
    tracking_lead = bool(getattr(self.starpilot_planner, "tracking_lead", False))
    set_speed_error = max(0.0, v_cruise - v_ego)

    if (not lead_status and not tracking_lead and
        v_ego >= starpilot_toggles.conditional_chill_speed and
        set_speed_error >= starpilot_toggles.conditional_chill_speed_margin):
      return CCStatus["SPEED"], False

    if not starpilot_toggles.conditional_chill_lead:
      return CCStatus["OFF"], False

    if v_ego < starpilot_toggles.conditional_chill_speed_lead or not lead_status or not tracking_lead:
      return CCStatus["OFF"], False

    lead_distance = float(getattr(lead, "dRel", float("inf")))
    lead_speed = float(getattr(lead, "vLead", 0.0))
    lead_brake = max(0.0, -float(getattr(lead, "aLeadK", 0.0)))
    lead_prob = float(getattr(lead, "modelProb", 0.0))
    closing_speed = max(0.0, v_ego - lead_speed)
    max_closing_speed = max(self.STABLE_LEAD_MAX_CLOSING_SPEED, self.STABLE_LEAD_MAX_CLOSING_RATIO * v_ego)
    max_distance = min(self.STABLE_LEAD_MAX_DISTANCE, max(35.0, v_ego * self.STABLE_LEAD_MAX_DISTANCE_TIME))
    lead_confident = bool(getattr(lead, "radar", False)) or lead_prob >= self.STABLE_LEAD_MIN_MODEL_PROB

    if not lead_confident:
      return CCStatus["OFF"], False

    if lead_distance >= max_distance or lead_speed <= self.STABLE_LEAD_MIN_SPEED:
      return CCStatus["OFF"], False

    if lead_brake > self.STABLE_LEAD_MAX_BRAKE or closing_speed > max_closing_speed:
      return CCStatus["OFF"], False

    return CCStatus["LEAD"], False

  def _get_launch_status(self, v_ego, sm):
    now = time.monotonic()
    standstill = bool(sm["carState"].standstill)
    stopped = standstill or v_ego < self.CHILL_LAUNCH_ARM_SPEED

    if self._launch_active:
      self._launch_distance += max(0.0, v_ego) * max(0.0, min(now - self._launch_last_time, 0.5))
      self._launch_last_time = now

      if v_ego >= self.CHILL_LAUNCH_EXIT_SPEED:
        # Launch done: hand back to the normal Chill rules without forcing experimental.
        self._end_launch()
        return CCStatus["OFF"]

      if stopped or self._launch_exit_required(sm):
        self._end_launch()
        self._launch_forced_exit = True
        self._launch_armed = stopped
        return CCStatus["OFF"]

      return self._get_launch_cc_status()

    if stopped:
      self._launch_armed = True
      return CCStatus["OFF"]

    if not self._launch_armed or v_ego < self.CHILL_LAUNCH_START_SPEED:
      return CCStatus["OFF"]

    selfdrive_state = self._get_sm_service(sm, "selfdriveState")
    if selfdrive_state is None or not bool(getattr(selfdrive_state, "enabled", False)):
      self._launch_armed = False
      return CCStatus["OFF"]

    self._launch_armed = False
    self._launch_forced_exit = False
    self._launch_active = True
    self._launch_distance = 0.0
    self._launch_last_time = now
    return self._get_launch_cc_status()

  def _end_launch(self):
    self._launch_active = False
    self._launch_distance = 0.0

  def _launch_has_stop_flag(self, sm):
    starpilot_plan = self._get_sm_service(sm, "starpilotPlan")
    vcruise = self.starpilot_planner.starpilot_vcruise
    flags = {
      "stop_light": self.detector.stop_light_detected,
      "stop_light_model": self.detector.stop_light_model_detected,
      "raw_model_stopped": self.starpilot_planner.raw_model_stopped,
      "model_stopped": self.starpilot_planner.model_stopped,
      "stop_sign": vcruise.stop_sign_confirmed,
      "forcing_stop": vcruise.forcing_stop,
      "red_light": bool(getattr(starpilot_plan, "redLight", False)),
      "plan_forcing_stop": bool(getattr(starpilot_plan, "forcingStop", False)),
    }
    return any(flags.values())

  def _launch_exit_required(self, sm):
    selfdrive_state = self._get_sm_service(sm, "selfdriveState")
    if selfdrive_state is None or not bool(getattr(selfdrive_state, "enabled", False)):
      return True

    if self._launch_distance < self.CHILL_LAUNCH_STOP_FLAG_DISTANCE:
      return False

    return self._launch_has_stop_flag(sm) or self._launch_lead_stopping()

  def _launch_lead_stopping(self):
    if self.detector.slow_lead_detected:
      return True

    lead = getattr(self.starpilot_planner, "lead_one", None)
    if not bool(getattr(lead, "status", False)):
      return False

    lead_speed = float(getattr(lead, "vLead", float("inf")))
    lead_accel = float(getattr(lead, "aLeadK", 0.0))
    return (
      lead_speed < self.CHILL_LAUNCH_LEAD_STOPPED_SPEED or
      (lead_speed < self.CHILL_LAUNCH_LEAD_STOPPING_SPEED and lead_accel <= self.CHILL_LAUNCH_LEAD_STOPPING_DECEL)
    )

  def _get_launch_cc_status(self):
    lead = getattr(self.starpilot_planner, "lead_one", None)
    lead_status = bool(getattr(lead, "status", False))
    tracking_lead = bool(getattr(self.starpilot_planner, "tracking_lead", False))
    return CCStatus["LEAD"] if lead_status or tracking_lead else CCStatus["SPEED"]

  def _get_entry_confirm_time(self, auto_status, launch_candidate):
    if launch_candidate:
      return self.CHILL_LAUNCH_ENTRY_CONFIRM_TIME

    if auto_status == CCStatus["LEAD"]:
      return self.CHILL_LEAD_ENTRY_CONFIRM_TIME

    return self.CHILL_SPEED_ENTRY_CONFIRM_TIME

  def _adjacent_lead_ambiguous(self, sm, v_ego):
    radar_state = self._get_sm_service(sm, "starpilotRadarState")
    if radar_state is None:
      return False

    max_distance = min(self.ADJACENT_LEAD_VETO_MAX_DISTANCE, max(25.0, v_ego * self.ADJACENT_LEAD_VETO_MAX_DISTANCE_TIME))
    for lead in (getattr(radar_state, "leadLeft", None), getattr(radar_state, "leadRight", None)):
      if lead is None or not getattr(lead, "status", False):
        continue

      lateral_offset = abs(float(getattr(lead, "yRel", 0.0)))
      if lateral_offset > self.ADJACENT_LEAD_VETO_MAX_LATERAL_OFFSET:
        continue

      if float(getattr(lead, "dRel", float("inf"))) < max_distance and float(getattr(lead, "vLead", 0.0)) > self.ADJACENT_LEAD_VETO_MIN_SPEED:
        return True

    return False

  @staticmethod
  def _get_sm_service(sm, key):
    if isinstance(sm, dict):
      return sm.get(key)

    try:
      return sm[key]
    except (KeyError, IndexError, TypeError, AttributeError):
      return None

  def _write_status(self, status_value):
    if status_value != self._prev_cc_status:
      self.params_memory.put_int("CCStatus", status_value)
      self._prev_cc_status = status_value
