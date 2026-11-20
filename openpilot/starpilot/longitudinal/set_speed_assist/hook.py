"""
Where the longitudinal planners meet Set Speed Assist.

Both planners (StarPilot's selfdrive/controls/lib/longitudinal_planner.py and the stock one in
starpilot/longitudinal/upstream/planner.py) call e2e_candidate() at the Accel Boost site, so the two
behave the same. With the SetSpeedAssist param off and no lift left to ease out, the call is exactly
the Accel Boost call the planner made before. With it on, Accel Boost is not run at all (its state does
not move and the planner publishes accelBoost = 0), and the e2e candidate is the Set Speed Assist lift
toward chill mode's answer.

Parts of this follow zoompilot's update_e2e_target (commit e6baf0b50, 2e9506a87); see NOTICE.md.
"""
from openpilot.starpilot.longitudinal.set_speed_assist.approach import approach_accel
from openpilot.starpilot.longitudinal.set_speed_assist.controller import SetSpeedAssistController


def raw_personality(personality) -> int:
  """The enum's raw value: a message reader gives an enum with .raw, a test may give the int."""
  return int(getattr(personality, 'raw', personality))


def e2e_candidate(assist: SetSpeedAssistController, accel_boost, sm, *, a_model: float, a_mpc: float,
                  a_cruise_prev: float, a_cruise: float, v_target: float, is_e2e: bool,
                  reset_state: bool) -> tuple[float, bool]:
  """The e2e candidate acceleration, and whether Set Speed Assist owns it this frame.

  a_model: the raw model acceleration Accel Boost would have boosted.
  a_mpc: the planner's MPC candidate, at the action time the planner uses for it.
  a_cruise_prev / a_cruise: the cruise candidate of the previous frame (chill's floor, as zoompilot
    takes it: the host builds this frame's after the e2e candidate) and of this frame (Accel Boost's input).
  v_target: the cruise target speed after every ceiling, the speed the cruise candidate aims at.
  is_e2e: the planner takes the e2e candidate into its minimum (experimentalMode).
  reset_state: the planner is not engaged or the driver is controlling the speed.

  The second value is True while the assist is on or still easing its lift out; then Accel Boost was
  not called and the planner must publish accelBoost = 0.
  """
  if not assist.poll():
    return accel_boost.update(sm, a_model, a_mpc, a_cruise), False
  v_ego = sm['carState'].vEgo
  # lateral acceleration from the measured steering: controlsd's curvature from the learned steer ratio,
  # angle offset and roll
  steer_lat_accel = sm['controlsState'].curvature * v_ego ** 2
  # chill mode's answer: the MPC candidate, and last frame's cruise candidate, capped at the approach pace
  a_chill = min(float(a_mpc), float(a_cruise_prev),
                approach_accel(v_ego, v_target, raw_personality(sm['selfdriveState'].personality)))
  # the planner's Accel Boost lead gate (Bosch-A Honda) also gates the lift
  lead_gate = bool(getattr(accel_boost, 'lead_gate', False))
  return assist.update(sm, a_model, a_chill, is_e2e, reset_state, steer_lat_accel, lead_gate), True
