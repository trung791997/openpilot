"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ported from zoompilot (e2e_set_speed tests, commits e6baf0b50 and 2e9506a87) as unittest, see ../NOTICE.md.

Closed loop against a model fitted from zoompilot's experimental-mode logs: it pulls toward its own
pace at about -0.02 m/s^2 per m/s (43 ACC set-speed steps, tau ~50 s), with a 0.5 s actuator lag.
The upper bound is this tree's cruise candidate, as in the planner with no lead, and the floor closes
on the set speed at the approach pace. These are simulations of the assist (static / unit test), not road evidence.
"""
import unittest
from types import SimpleNamespace

import numpy as np

from openpilot.cereal import log as cereal_log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL as DT
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel
from openpilot.starpilot.longitudinal.set_speed_assist.approach import approach_accel, approach_ceiling
from openpilot.starpilot.longitudinal.set_speed_assist.controller import LIFT_FALL, SetSpeedAssistController, op_long_cruise_jerk
from openpilot.starpilot.longitudinal.set_speed_assist.tests.helpers import ENGAGED, FakeParams, build_sm, plan_ramp

V_PREF = 20.  # m/s, the model's own pace
V_CRUISE = V_PREF + 6 * CV.MPH_TO_MS  # set 6 mph above it, the median gap in the logs
STANDARD = cereal_log.LongitudinalPersonality.standard
ACTUATOR_TAU = 0.5  # s
CP = SimpleNamespace(steerRatio=15., wheelbase=2.7)
# The model's acceleration wanders around its pull near the set speed: logged sd 0.08 m/s^2 below
# 0.3 Hz (open road within 1 m/s of the target). Three slow tones stand in for it.
WANDER = ((0.065, 7.), (0.065, 13.), (0.065, 29.))  # (m/s^2, period s)


def cruise_candidate(v, a_cruise_prev, a_target_prev):
  return get_cruise_accel(False, V_CRUISE, v, a_cruise_prev, 0., CP, DT, -0.5, True, a_target_prev=a_target_prev)


def simulate(enabled, seconds=60., tau=50., brake_at=None, coast_at=None, plan_drop=4., v0=V_PREF, wander=False):
  """Rows of (t, v_ego, a_model, e2e candidate, output). brake_at: the model brakes at -0.5 from then on.
  coast_at: from then on the model eases off gently, inside the gain's band, toward a stop its plan
  shows plan_drop m/s under the current speed at the end of the horizon. wander: add WANDER."""
  ctl = SetSpeedAssistController(params=FakeParams(SetSpeedAssist=enabled), dt=DT)
  v, a_act, a_cruise, out = v0, 0., 0., 0.
  log = []
  for i in range(int(seconds / DT)):
    t = i * DT
    a_model = float(np.clip((V_PREF - v) / tau, -1., 1.))
    if wander:
      a_model += float(sum(amp * np.sin(2 * np.pi * t / period) for amp, period in WANDER))
    if brake_at is not None and t >= brake_at:
      a_model = -0.5
    plan_v = None
    if coast_at is not None and t >= coast_at:
      a_model = -0.05
      plan_v = plan_ramp(v, plan_drop)
    # the hook passes last frame's cruise candidate at the approach pace, then the planner builds this frame's
    e2e = ctl.update(build_sm(v, plan_v=plan_v), a_model, min(a_cruise, approach_accel(v, V_CRUISE, STANDARD)), **ENGAGED)
    a_cruise = cruise_candidate(v, a_cruise, out)
    out = min(e2e, a_cruise)
    a_act += DT / ACTUATOR_TAU * (out - a_act)
    v += a_act * DT
    log.append((t, v, a_model, e2e, out))
  return np.asarray(log)


def deficit_mph(log, after):
  return (V_CRUISE - log[log[:, 0] >= after, 1]).max() * CV.MS_TO_MPH


def flips(command, deadband=0.05):
  """Throttle-brake changes: sign changes of the planner's command outside a small deadband."""
  signs = np.sign(command[np.abs(command) > deadband])
  return int(np.count_nonzero(np.diff(signs)))


class TestClosedLoop(unittest.TestCase):
  def test_model_alone_stays_under(self):
    self.assertGreater(deficit_mph(simulate(False), after=40.), 5.)

  def test_reaches_set_speed(self):
    for tau, deficit in ((25., 1.), (50., 1.), (65., 1.)):  # the per-bundle spread in the logs
      with self.subTest(tau=tau):
        log = simulate(True, tau=tau)
        self.assertLess(deficit_mph(log, after=40.), deficit)
        self.assertLess(deficit_mph(log, after=40.), 0.6 * deficit_mph(simulate(False, tau=tau), after=40.))
        # arrives without passing the set speed
        self.assertLess(log[:, 1].max() - V_CRUISE, 0.02)

  def test_added_jerk_bounded(self):
    # the model never brakes in this run, so the lift never takes the gain's faster drop
    log = simulate(True)
    lift = log[:, 3] - log[:, 2]
    jerk = np.diff(lift) / DT
    self.assertGreaterEqual(jerk.min(), -LIFT_FALL - 1e-6)
    self.assertLessEqual(jerk.max(), op_long_cruise_jerk(0.) + 1e-6)
    self.assertGreaterEqual(lift.min(), 0.)

  def test_yields_to_a_slowdown_within_half_a_second(self):
    log = simulate(True, brake_at=45.)
    after = log[log[:, 0] >= 45. + 0.5]
    self.assertTrue(np.allclose(after[:, 3], after[:, 2]))

  def test_coasts_toward_a_stop_the_plan_shows(self):
    for plan_drop in (3., 6.):
      with self.subTest(plan_drop=plan_drop):
        log = simulate(True, coast_at=45., plan_drop=plan_drop)
        after = log[log[:, 0] >= 45.]
        self.assertTrue(np.allclose(after[:, 3], after[:, 2]))

  def test_open_road_plan_keeps_the_lift(self):
    log = simulate(True, coast_at=45., plan_drop=0.)
    after = log[log[:, 0] >= 50.]
    self.assertGreater((after[:, 3] - after[:, 2]).min(), 0.)

  def test_holds_the_set_speed_without_oscillating(self):
    for wander in (False, True):
      with self.subTest(wander=wander):
        log = simulate(True, seconds=160., v0=V_CRUISE - 0.3, wander=wander)
        hold = log[log[:, 0] >= 40.]
        self.assertLess(np.median(V_CRUISE - hold[:, 1]), 0.1)
        self.assertLess((V_CRUISE - hold[:, 1]).max(), 0.3)
        self.assertLess((hold[:, 1] - V_CRUISE).max(), 0.1)
        self.assertLessEqual(flips(hold[:, 4]), flips(hold[:, 2]) if wander else 1)

  def test_closes_a_shortfall_without_overshoot(self):
    log = simulate(True, seconds=40., v0=V_CRUISE - 5.)
    within = log[np.abs(V_CRUISE - log[:, 1]) < 0.2, 0]
    self.assertLess(within[0], 13.)
    self.assertLess(log[:, 1].max() - V_CRUISE, 0.02)
    self.assertLessEqual(log[:, 4].max(), approach_ceiling(V_CRUISE - 5., STANDARD) + 1e-6)


if __name__ == "__main__":
  unittest.main()
