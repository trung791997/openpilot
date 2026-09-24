import math

from cereal import car
from opendbc.car import apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET

ButtonType = car.CarState.ButtonEvent.Type

SEND_BUTTON_NONE = 0
SEND_BUTTON_INCREASE = 1
SEND_BUTTON_DECREASE = 2

HYST_GAP = 0.0
INCREASE_INACTIVE_TIMER = 0.12
DECREASE_INACTIVE_TIMER = 0.05
LEAD_INCREASE_INACTIVE_TIMER = 0.05
MANUAL_BUTTON_INACTIVE_TIMER = 0.5
LEAD_RECOVERY_LOOKAHEAD_POINTS = 4
LEAD_RECOVERY_HOLD_BUFFER_MS = 1.5 * CV.MPH_TO_MS
LEAD_COAST_BUFFER_MS = 1.0 * CV.MPH_TO_MS
LEAD_EXTRA_COAST_BUFFER_FACTOR = 0.6
LEAD_EXTRA_COAST_BUFFER_MAX_MS = 3.0 * CV.MPH_TO_MS
LEAD_EXTRA_COAST_HEADWAY_MIN_S = 1.5
LEAD_EXTRA_COAST_HEADWAY_MAX_S = 3.0
LEAD_CLOSING_REL_SPEED_MIN_MS = 0.5 * CV.MPH_TO_MS
LEAD_PROACTIVE_COAST_HEADWAY_MAX_S = 4.0
LEAD_DEPARTURE_REL_SPEED_MIN_MS = 1.0 * CV.MPH_TO_MS
LEAD_DEPARTURE_HEADWAY_MIN_S = 1.8
LEAD_DEPARTURE_HEADWAY_MAX_S = 4.5
LEAD_DEPARTURE_BOOST_MIN_MS = 1.25 * CV.MPH_TO_MS
LEAD_DEPARTURE_BOOST_MAX_MS = 3.0 * CV.MPH_TO_MS
LEAD_DEPARTURE_BOOST_FACTOR = 0.50
LEAD_DEPARTURE_PLAN_POINTS = 3

# Far lead (stock ACC only, behind ICBMFarLead): the chill planner's speeds are ICBM's only lead input
# and they do not come down for a slow or stopped lead 100 m out (route 25e 723-727 s, replay: the plan
# stayed at the set speed until the lead was 83 m away, then wanted 22 -> 8 m/s in 3 s and the car's own
# ACC braked -3.5). This target is the speed from which a constant FAR_LEAD_DECEL_MS2 stop reaches the
# lead's speed with FAR_LEAD_HEADWAY_S of headway (never under FAR_LEAD_MIN_GAP_M). It is taken as the
# minimum with the plan-derived target, so it only ever lowers the set speed. 1.5 m/s^2 is what the
# set speed can follow at ~2 steps/s (1 mph/step) without the stock ACC having to brake hard. Replay
# evidence only, never driven.
FAR_LEAD_DECEL_MS2 = 1.5
FAR_LEAD_MIN_GAP_M = 6.0
FAR_LEAD_HEADWAY_S = 1.5

# Launch (stock ACC only): after a full stop, raise the set speed straight to the cruise target once the
# car is moving again, instead of holding at the 25 mph floor until vEgo passes it. Route 260 8:39-8:51:
# the car launched to 25 mph with no ICBM press for 11 s, while the lead pulled away from 8 to 34 m.
# Stock ACC keeps the radar and does the following, so a set speed above the lead's speed does not close
# on it. Ends when the car has caught up to the target, stops again, or the driver takes over.
LAUNCH_STOPPED_SPEED_MS = 0.3
LAUNCH_MOVING_SPEED_MS = 2.0 * CV.MPH_TO_MS
LAUNCH_CAUGHT_UP_MARGIN_MS = 2.0 * CV.MPH_TO_MS
# Route 262 3:43-4:26: the launch raised the set speed to 49.7 mph by 3:50, then held it for 36 s while
# the lead drove 30-45 mph, because it only ended at vEgo >= target - 2 mph. It now also ends once the set
# speed has been raised and the car has caught up to the normal (lead or plan) target, so ICBM follows the
# lead again.

# Gas release (SetSpeedOnGasRelease): when the driver releases the gas above the set speed on the dash,
# hold the ICBM target at or above the release speed until a cruise button, the brake, a stop or a
# disengage. Route 262 1:29 and 2:25: the old path moved openpilot's v_cruise (the 55 mph ICBM max), not
# the dash set speed, so a release at 37 mph over a 24.9 mph set speed changed nothing.
GAS_RELEASE_SET_MARGIN_MS = 1.0 * CV.MPH_TO_MS
# Gas snap: the floor alone ramps the set speed at 1 mph per press after the release (route 263 13:44:
# 26 -> 41 mph took 3.1 s while the car slowed 42.4 -> 40.4 mph), so while the gas is still held ICBM
# pulses DECEL_SET, which on a Honda snaps the set speed to vEgo under gas (route 260 seg 9: 29 -> 32 mph).
# Only above set + 1 mph (Honda: set >= 25 mph, so never at low speed, where DECEL_SET would set 25) and
# not above the cruise target (vCruise, SLC, CSC), so it only ever raises the set speed to where the
# driver already is.
GAS_SNAP_PRESS_S = 0.2
GAS_SNAP_INTERVAL_S = 0.6

HONDA_MINIMUM_SET_SPEED_MPH = 25
# Honda reports the set speed in whole km/h, truncated: 54 mph -> 86 km/h -> 53.4 mph, which rounds to 53. With a
# 54 mph target, route 263 10:05-10:27 hunted +/- between 85, 86 and 88 km/h (~40 presses in 5 s, the dash beeps).
# The truncation loses up to 0.62 mph, so adding half of that before rounding recovers the whole mph the car holds.
HONDA_KPH_TRUNCATION_MPH = 0.5 * CV.KPH_TO_MPH
HONDA_MINIMUM_SET_SPEED_KPH = 40

CRUISE_BUTTON_TIMERS = {
  int(ButtonType.decelCruise): 0,
  int(ButtonType.accelCruise): 0,
  int(ButtonType.setCruise): 0,
  int(ButtonType.resumeCruise): 0,
  int(ButtonType.cancel): 0,
  int(ButtonType.mainCruise): 0,
}


def select_redneck_target_speed(v_cruise_kph: float, speed_cluster_ms: float,
                                starpilot_target_speed_ms: float, plan_speeds_ms: list[float],
                                lookahead_points: int, allow_plan_decrease: bool = True,
                                lead_present: bool = False, lead_distance_m: float = 0.0,
                                lead_rel_speed_ms: float = 0.0,
                                lead_speed_ms: float | None = None,
                                slc_target_speed_ms: float = 0.0,
                                csc_target_speed_ms: float = 0.0) -> float:
  target_speed_ms = float(speed_cluster_ms)
  if slc_target_speed_ms > 0:
    target_speed_ms = float(slc_target_speed_ms)
    # SLC is an upper bound for the button-spammed stock setpoint. A driver-set
    # speed below the posted target must still be able to slow the car down.
    if 0 < v_cruise_kph < V_CRUISE_UNSET:
      target_speed_ms = min(target_speed_ms, float(v_cruise_kph) * CV.KPH_TO_MS)
  elif v_cruise_kph > 0:
    target_speed_ms = float(v_cruise_kph) * CV.KPH_TO_MS
  elif starpilot_target_speed_ms > 0:
    target_speed_ms = float(starpilot_target_speed_ms)

  # Curve Speed Control publishes its own target (starpilotPlan.cscSpeed). The branches above
  # use the driver set speed or SLC and never see it, so apply it here as a cap only.
  if csc_target_speed_ms > 0:
    target_speed_ms = min(target_speed_ms, float(csc_target_speed_ms))

  lead_closing = lead_present and lead_rel_speed_ms < -LEAD_CLOSING_REL_SPEED_MIN_MS
  # ICBMFarLead: inf (no effect) unless card.py passed the lead speed and the lead is closing.
  far_lead_target_ms = float("inf")
  if lead_closing and lead_speed_ms is not None:
    far_lead_target_ms = get_far_lead_target_ms(lead_distance_m, lead_speed_ms)

  if allow_plan_decrease and len(plan_speeds_ms) > 0:
    if lead_present and not lead_closing and target_speed_ms > speed_cluster_ms and plan_speeds_ms[0] > speed_cluster_ms:
      recovery_lookahead_points = min(len(plan_speeds_ms), LEAD_RECOVERY_LOOKAHEAD_POINTS)
      recovery_target_speed_ms = max(speed_cluster_ms, min(plan_speeds_ms[:recovery_lookahead_points]))
      departure_boost_ms = get_lead_departure_boost_ms(
        speed_cluster_ms,
        lead_distance_m,
        lead_rel_speed_ms,
        plan_speeds_ms,
      )
      if departure_boost_ms > 0.0:
        recovery_target_speed_ms = max(recovery_target_speed_ms, speed_cluster_ms + departure_boost_ms)
      return min(target_speed_ms, recovery_target_speed_ms, far_lead_target_ms)

    decrease_target_speed_ms = min(plan_speeds_ms[:lookahead_points])
    lead_headway_s = lead_distance_m / speed_cluster_ms if lead_distance_m > 0.0 and speed_cluster_ms > 0.1 else float("inf")
    proactive_coast = lead_closing and lead_headway_s <= LEAD_PROACTIVE_COAST_HEADWAY_MAX_S

    if not proactive_coast and lead_present and target_speed_ms > speed_cluster_ms and \
        decrease_target_speed_ms >= speed_cluster_ms - LEAD_RECOVERY_HOLD_BUFFER_MS:
      return min(speed_cluster_ms, far_lead_target_ms)

    if lead_present and (decrease_target_speed_ms < speed_cluster_ms or proactive_coast):
      decrease_target_speed_ms = max(0.0, decrease_target_speed_ms - get_lead_coast_buffer_ms(
        speed_cluster_ms,
        lead_distance_m,
        lead_rel_speed_ms,
      ))

    if proactive_coast and target_speed_ms > speed_cluster_ms and \
        decrease_target_speed_ms >= speed_cluster_ms - LEAD_RECOVERY_HOLD_BUFFER_MS:
      return min(speed_cluster_ms, far_lead_target_ms)

    if decrease_target_speed_ms < target_speed_ms:
      return min(decrease_target_speed_ms, far_lead_target_ms)

  return min(target_speed_ms, far_lead_target_ms)


def get_lead_coast_buffer_ms(speed_cluster_ms: float, lead_distance_m: float, lead_rel_speed_ms: float) -> float:
  lead_closing_speed_ms = max(-float(lead_rel_speed_ms), 0.0)
  if lead_closing_speed_ms <= 0.0:
    return LEAD_COAST_BUFFER_MS

  headway_factor = 0.0
  if lead_distance_m > 0.0 and speed_cluster_ms > 0.1:
    headway_s = lead_distance_m / speed_cluster_ms
    headway_factor = min(max(
      (LEAD_EXTRA_COAST_HEADWAY_MAX_S - headway_s) /
      (LEAD_EXTRA_COAST_HEADWAY_MAX_S - LEAD_EXTRA_COAST_HEADWAY_MIN_S),
      0.0,
    ), 1.0)

  extra_buffer_ms = min(LEAD_EXTRA_COAST_BUFFER_MAX_MS, lead_closing_speed_ms * LEAD_EXTRA_COAST_BUFFER_FACTOR)
  return LEAD_COAST_BUFFER_MS + extra_buffer_ms * (0.5 + (0.5 * headway_factor))


def get_far_lead_target_ms(lead_distance_m: float, lead_speed_ms: float) -> float:
  """Speed from which a FAR_LEAD_DECEL_MS2 decel reaches lead_speed_ms at the desired gap. inf if no lead."""
  if lead_distance_m <= 0.0:
    return float("inf")
  lead_speed_ms = max(float(lead_speed_ms), 0.0)
  gap_m = max(FAR_LEAD_MIN_GAP_M, FAR_LEAD_HEADWAY_S * lead_speed_ms)
  return math.sqrt(max(0.0, lead_speed_ms ** 2 + 2.0 * FAR_LEAD_DECEL_MS2 * (lead_distance_m - gap_m)))


def get_lead_departure_boost_ms(speed_cluster_ms: float, lead_distance_m: float, lead_rel_speed_ms: float,
                                plan_speeds_ms: list[float]) -> float:
  if lead_rel_speed_ms < LEAD_DEPARTURE_REL_SPEED_MIN_MS or speed_cluster_ms <= 0.1 or lead_distance_m <= 0.0:
    return 0.0

  plan_points = min(len(plan_speeds_ms), LEAD_DEPARTURE_PLAN_POINTS)
  if plan_points <= 0 or min(plan_speeds_ms[:plan_points]) <= speed_cluster_ms:
    return 0.0

  headway_s = lead_distance_m / speed_cluster_ms
  if headway_s < LEAD_DEPARTURE_HEADWAY_MIN_S:
    return 0.0

  headway_factor = min(max(
    (headway_s - LEAD_DEPARTURE_HEADWAY_MIN_S) /
    (LEAD_DEPARTURE_HEADWAY_MAX_S - LEAD_DEPARTURE_HEADWAY_MIN_S),
    0.0,
  ), 1.0)
  extra_boost_ms = max(lead_rel_speed_ms - LEAD_DEPARTURE_REL_SPEED_MIN_MS, 0.0) * LEAD_DEPARTURE_BOOST_FACTOR
  boost_ms = LEAD_DEPARTURE_BOOST_MIN_MS + extra_boost_ms * (0.5 + (0.5 * headway_factor))
  return min(boost_ms, LEAD_DEPARTURE_BOOST_MAX_MS)


def get_minimum_set_speed(is_metric: bool, brand: str = "") -> int:
  # Honda stock ACC will not hold a set speed below 25 mph, so asking for less
  # just spams DECEL_SET into a floor the car ignores.
  if brand == "honda":
    return HONDA_MINIMUM_SET_SPEED_KPH if is_metric else HONDA_MINIMUM_SET_SPEED_MPH
  return 30 if is_metric else 20


def update_launch_state(launch_active: bool, was_stopped: bool, enabled: bool, v_ego: float, standstill: bool,
                        gas_pressed: bool, driver_button: bool, launch_target_ms: float,
                        set_speed_ms: float | None = None, hold_target_ms: float | None = None) -> tuple[bool, bool]:
  """Returns (launch_active, was_stopped). A launch never starts while the car is stopped, so ICBM does
  not press RES+ at standstill (on a Honda that resumes the car by itself)."""
  if not enabled or driver_button:
    return False, False
  if standstill or v_ego < LAUNCH_STOPPED_SPEED_MS:
    return False, True
  if was_stopped and v_ego >= LAUNCH_MOVING_SPEED_MS and not gas_pressed:
    launch_active, was_stopped = True, False
  if launch_active and v_ego >= launch_target_ms - LAUNCH_CAUGHT_UP_MARGIN_MS:
    launch_active = False
  if launch_active and set_speed_ms is not None and hold_target_ms is not None and \
      set_speed_ms >= launch_target_ms - LAUNCH_CAUGHT_UP_MARGIN_MS and \
      v_ego >= hold_target_ms - LAUNCH_CAUGHT_UP_MARGIN_MS:
    launch_active = False
  return launch_active, was_stopped


def update_gas_release_floor(floor_ms: float, gas_pressed_prev: bool, gas_pressed: bool, enabled: bool, v_ego: float,
                             standstill: bool, brake_pressed: bool, driver_button: bool, set_speed_ms: float,
                             is_metric: bool, snapped: bool = False) -> float:
  """Returns the ICBM target floor in m/s (0 = none). The caller caps it at the cruise target. snapped: a gas
  snap raised the set speed during this press, so the release is floored even with the set already near vEgo."""
  if not enabled or driver_button or brake_pressed or standstill or v_ego < LAUNCH_STOPPED_SPEED_MS:
    return 0.0
  if gas_pressed_prev and not gas_pressed and (snapped or v_ego > set_speed_ms + GAS_RELEASE_SET_MARGIN_MS):
    if is_metric:
      return round(v_ego * CV.MS_TO_KPH) * CV.KPH_TO_MS
    return round(v_ego * CV.MS_TO_MPH) * CV.MPH_TO_MS
  return floor_ms


def want_gas_snap(enabled: bool, gas_pressed: bool, driver_button: bool, brake_pressed: bool, v_ego: float,
                  set_speed_ms: float, cap_ms: float) -> bool:
  return enabled and gas_pressed and not driver_button and not brake_pressed and \
    set_speed_ms + GAS_RELEASE_SET_MARGIN_MS < v_ego <= cap_ms


def update_manual_button_timers(CS: car.CarState, button_timers: dict[int, int]) -> None:
  for button_type in button_timers:
    if button_timers[button_type] > 0:
      button_timers[button_type] += 1

  for event in CS.buttonEvents:
    button_type = event.type.raw if hasattr(event.type, "raw") else int(event.type)
    if button_type in button_timers:
      button_timers[button_type] = 1 if event.pressed else 0


class RedneckCruise:
  def __init__(self, CP, FPCP):
    self.CP = CP
    self.FPCP = FPCP

    self.v_target = 0
    self.v_target_ms_last = 0.0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.state = "inactive"
    self.pre_active_timer = 0
    self.is_ready = False
    self.is_ready_prev = False
    self.cruise_button_timers = dict(CRUISE_BUTTON_TIMERS)
    self.manual_button_pressed = False
    self.gas_snap_frame = 0

  @staticmethod
  def _send_button_for_state(state: str) -> int:
    if state == "increasing":
      return SEND_BUTTON_INCREASE
    if state == "decreasing":
      return SEND_BUTTON_DECREASE
    return SEND_BUTTON_NONE

  def _reset(self) -> None:
    self.state = "inactive"
    self.pre_active_timer = 0
    self.is_ready = False
    self.is_ready_prev = False

  def _update_calculations(self, CS: car.CarState, v_target_ms: float, is_metric: bool) -> None:
    speed_conv = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS

    self.v_target_ms_last = apply_hysteresis(v_target_ms, self.v_target_ms_last, HYST_GAP * ms_conv)
    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(is_metric, getattr(self.CP, "brand", ""))
    cluster = CS.cruiseState.speedCluster * speed_conv
    if not is_metric and getattr(self.CP, "brand", "") == "honda":
      cluster += HONDA_KPH_TRUNCATION_MPH
    self.v_cruise_cluster = round(cluster)

  def _update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)
    button_pressed = any(0 < timer <= int(MANUAL_BUTTON_INACTIVE_TIMER / DT_CTRL) for timer in self.cruise_button_timers.values())
    self.manual_button_pressed = button_pressed
    # cruiseControl.override is only set under openpilot long, so the stock-ACC gas override is
    # checked directly. Honda DECEL/SET under gas snaps the set speed to vEgo (route 260 seg 6/9:
    # 39 -> 25 mph at 14 mph, 29 -> 32 mph while ICBM was decreasing).
    self.is_ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume and \
      not button_pressed and not getattr(CS, "gasPressed", False)

  def _desired_state(self) -> str:
    if self.v_target > self.v_cruise_cluster:
      return "increasing"
    if self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
      return "decreasing"
    return "holding"

  @staticmethod
  def _get_pre_active_frames(state: str, lead_present: bool) -> int:
    if state == "decreasing":
      timer = DECREASE_INACTIVE_TIMER
    elif lead_present:
      timer = LEAD_INCREASE_INACTIVE_TIMER
    else:
      timer = INCREASE_INACTIVE_TIMER
    return int(timer / DT_CTRL)

  def _arm_pre_active(self, desired_state: str, lead_present: bool) -> None:
    if desired_state == "holding":
      self.state = "holding"
      self.pre_active_timer = 0
      return

    self.state = "preActive"
    self.pre_active_timer = self._get_pre_active_frames(desired_state, lead_present)

  def _update_state_machine(self, lead_present: bool) -> int:
    desired_state = self._desired_state()

    if not self.is_ready:
      self.state = "inactive"
      self.pre_active_timer = 0
    elif self.state == "inactive":
      if not self.is_ready_prev:
        self._arm_pre_active(desired_state, lead_present)
    elif self.state == "preActive":
      if desired_state == "holding":
        self.state = "holding"
        self.pre_active_timer = 0
      else:
        desired_frames = self._get_pre_active_frames(desired_state, lead_present)
        self.pre_active_timer = max(0, min(self.pre_active_timer, desired_frames) - 1)
        if self.pre_active_timer <= 0:
          self.state = desired_state
    elif self.state == "holding":
      if desired_state != "holding":
        self._arm_pre_active(desired_state, lead_present)
    elif self.state != desired_state:
      if desired_state == "holding":
        self.state = "holding"
      else:
        self._arm_pre_active(desired_state, lead_present)

    return self._send_button_for_state(self.state)

  def _gas_snap_button(self, CS: car.CarState, CC: car.CarControl, gas_snap: bool) -> int:
    # Pulsed, so a press never lands after the gas is released (without gas, DECEL_SET lowers the set speed).
    if not gas_snap or not getattr(CS, "gasPressed", False) or not CC.enabled or CC.cruiseControl.cancel or \
        CC.cruiseControl.resume or self.manual_button_pressed:
      self.gas_snap_frame = 0
      return SEND_BUTTON_NONE
    phase = self.gas_snap_frame % int(GAS_SNAP_INTERVAL_S / DT_CTRL)
    self.gas_snap_frame += 1
    return SEND_BUTTON_DECREASE if phase < int(GAS_SNAP_PRESS_S / DT_CTRL) else SEND_BUTTON_NONE

  def run(self, CS: car.CarState, CC: car.CarControl, v_target_ms: float, is_metric: bool,
          lead_present: bool = False, gas_snap: bool = False) -> tuple[int, int]:
    if self.FPCP.pcmCruiseSpeed or not self.FPCP.redneckCruiseAvailable:
      self._reset()
      return SEND_BUTTON_NONE, 0

    self._update_calculations(CS, v_target_ms, is_metric)
    self._update_readiness(CS, CC)
    send_button = self._update_state_machine(lead_present)
    snap_button = self._gas_snap_button(CS, CC, gas_snap)
    if snap_button != SEND_BUTTON_NONE:
      send_button = snap_button

    self.is_ready_prev = self.is_ready
    return send_button, self.v_target
