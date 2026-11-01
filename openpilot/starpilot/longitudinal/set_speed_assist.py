"""Set Speed Assist: in Experimental, let the car reach the set speed and the personality's gap when the model runs slow.

Port of sunnypilot PR #2083 (Zeph Leggett, MIT, open 2026-10-29, 8341d270b1). The driving model has no set-speed
input, so with nothing ahead the e2e candidate holds its own pace, often several mph under the set speed, and behind a
lead it can hang back past the personality's gap. This lifts the e2e candidate toward min(lead MPC, cruise) while the
model is not slowing: it fades as the model's accel drops from -0.1 to -0.3, scales with the model's gas-press
probability, is off below 5 m/s (full by 8), fades for a bend ahead (0.6 -> 1.0 m/s^2 lateral) and is off in a lane
change. It builds at the cruise jerk and drops at 2.4 m/s^3, so it never holds off the model's braking. It never
lowers the candidate and never lifts above the lead/cruise candidates.

Differences from the PR: no Params toggle (owner 2026-11-01 "let's try it"; on for radar cars), and vision-only cars are
left alone (owner 2026-10-19: vision-only + Experimental should track the model's own accel). Replay evidence pending.
"""
import math

import numpy as np

from openpilot.common.realtime import DT_MDL

MIN_SPEED = 5.  # m/s. No assist below this, so stop-and-go stays with the model.
FULL_SPEED = 8.  # m/s. Full assist from this speed up.
GAIN_BP = [-0.3, -0.1]  # model accel over which the assist fades out as the model slows down
FADE_LAT_ACC_BP = [0.6, 1.0]  # lateral accel of the sharpest bend ahead over which the assist fades out
LIFT_FALL = 2.4  # m/s^3. Fast, so the assist never holds off the model's braking.
LANE_CHANGE_OFF = 0  # log.LaneChangeState.off


class SetSpeedAssist:
  def __init__(self, CP, rise_bp, rise_vals, dt: float = DT_MDL):
    self.enabled = not bool(getattr(CP, 'radarUnavailable', True))
    self.rise_bp, self.rise_vals = rise_bp, rise_vals
    self.dt = dt
    self.active = False
    self.gain = 0.
    self.fade = 0.
    self.lift = 0.

  def update(self, sm, a_model: float, a_floor: float, reset_state: bool) -> float:
    """Return the model's accel plus the assist. a_floor is the lower of the lead and cruise accels."""
    self.active = False
    self.gain = self.fade = 0.
    if reset_state or not self.enabled:
      self.lift = 0.
      return a_model

    md = sm['modelV2']
    v_ego = float(sm['carState'].vEgo)
    vel = np.asarray(md.velocity.x, dtype=float)
    yaw_rate = np.asarray(md.orientationRate.z, dtype=float)
    gas_press_probs = md.meta.disengagePredictions.gasPressProbs
    throttle_prob = float(gas_press_probs[1]) if len(gas_press_probs) > 1 else 1.
    curvatures = [float(md.action.desiredCurvature), float(sm['controlsState'].curvature)]
    if not (len(vel) > 0 and len(vel) == len(yaw_rate) and
            np.isfinite(np.concatenate([vel, yaw_rate, [v_ego, a_model, a_floor, throttle_prob, *curvatures]])).all()):
      self.lift = 0.
      return a_model

    # sharpest bend now or along the model's path, at the current speed
    curvature = max(*np.abs(curvatures), float(np.max(np.abs(yaw_rate) / np.maximum(vel, 1.))))
    lat_acc = curvature * v_ego ** 2

    self.gain = float(np.interp(a_model, GAIN_BP, [0., 1.]) * throttle_prob * np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.]))
    self.fade = float(np.interp(lat_acc, FADE_LAT_ACC_BP, [1., 0.]))
    self.active = int(md.meta.laneChangeState) == LANE_CHANGE_OFF

    target = self.gain * self.fade * max(0., a_floor - a_model) if self.active else 0.
    rise = float(np.interp(v_ego, self.rise_bp, self.rise_vals))
    self.lift = min(max(target, self.lift - LIFT_FALL * self.dt), self.lift + rise * self.dt)
    if not math.isfinite(self.lift):
      self.lift = 0.
    return a_model + self.lift
