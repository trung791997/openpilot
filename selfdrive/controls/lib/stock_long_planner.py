#!/usr/bin/env python3
"""Camera-only Honda planner: the upstream openpilot planner as mvl-boston sp-honda-dev-202608 runs it.

Owner, 2026-10-19: on vision-only ACC the car should follow the driving model, since upstream openpilot has made
experimental mode its default and the models keep getting better. Keep the mvl live gas learner and mvl's Honda tuning
(both live in opendbc's Honda carcontroller/interface, untouched here); drop every other planner guard. plannerd picks
this planner when uses_stock_long_planner(CP); radar drives keep LongitudinalPlanner.

The output is min(lead MPC, jerk-limited cruise accel, model desiredAcceleration in experimental mode), as upstream.
Inputs from StarPilot: starpilotPlan.vCruise (set speed after speed-limit and curve control) and starpilotPlan.tFollow
(the driver's following distance). Static and unit evidence only; not driven.
"""
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.starpilot.common.starpilot_variables import get_longitudinal_actuator_delay

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
MIN_STABLE_DELAY = 0.3
# The MPC here only follows leads; cruise is the separate a_cruise candidate, as upstream.
MPC_V_CRUISE_LEADS_ONLY = V_CRUISE_MAX * CV.KPH_TO_MS

_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def uses_stock_long_planner(CP) -> bool:
  return getattr(CP, "brand", "") == "honda" and bool(getattr(CP, "radarUnavailable", False))


def should_stop(v_ego: float, a_target: float) -> bool:
  return bool(v_ego < 0.3 and a_target < 0.1)


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL):
  if len(speeds) != len(t_idxs):
    return 0.0
  v_now = speeds[0]
  a_now = accels[0]
  if action_t < MIN_STABLE_DELAY:
    v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
  else:
    v_target = np.interp(action_t, t_idxs, speeds)
  return float(2 * (v_target - v_now) / action_t - a_now)


def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  return float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))


class StockLongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.mpc.mode = 'acc'
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False
    self.source = 'cruise'

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm, starpilot_toggles=None):
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise = float(sm['starpilotPlan'].vCruise)
    if not np.isfinite(v_cruise):
      v_cruise = max(v_ego, 0.0)
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or sm['carState'].vCruise == V_CRUISE_UNSET

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = float(np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX))
      self.a_cruise = self.output_a_target

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    prev_accel_constraint = not (reset_state or sm['carState'].standstill)
    personality = sm['selfdriveState'].personality
    t_follow = float(sm['starpilotPlan'].tFollow)

    self.mpc.set_weights(prev_accel_constraint=prev_accel_constraint, personality=personality, v_ego=v_ego)
    self.mpc.set_accel_limits(ACCEL_MIN, ACCEL_MAX)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    zeros = np.zeros(len(T_IDXS_MPC))
    self.mpc.update(sm['radarState'], MPC_V_CRUISE_LEADS_ONLY, zeros.copy(), zeros.copy(), zeros.copy(), zeros.copy(),
                    1.0, t_follow, personality=personality, tracking_lead=True, optional_far_lead_comfort=False)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.output_a_target

    delay = get_longitudinal_actuator_delay(self.CP, starpilot_toggles) if starpilot_toggles is not None \
      else float(self.CP.longitudinalActuatorDelay)
    action_t = max(DT_MDL, delay) + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = float(sm['modelV2'].action.desiredAcceleration)
    output_should_stop_e2e = bool(sm['modelV2'].action.shouldStop)

    is_e2e = bool(sm['selfdriveState'].experimentalMode)

    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego, self.a_cruise, steer_angle_without_offset, self.CP,
                                     self.dt, accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, 'cruise', cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, 'e2e', output_should_stop_e2e))

    output_a_target, self.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(c[2] for c in candidates)
    self.output_a_target = float(np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX))

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    selected_lead = sm['radarState'].leadTwo if self.source == "lead1" else sm['radarState'].leadOne
    longitudinalPlan.hasLead = bool(selected_lead.status)
    longitudinalPlan.longitudinalPlanSource = self.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
