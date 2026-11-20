"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ported from zoompilot (e2e_set_speed tests, commits e6baf0b50 and 2e9506a87) as unittest, see ../NOTICE.md.
"""
import unittest

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL as DT
from openpilot.starpilot.longitudinal.set_speed_assist import approach
from openpilot.starpilot.longitudinal.set_speed_assist import controller as c
from openpilot.starpilot.longitudinal.set_speed_assist.controller import Inhibit, SetSpeedAssistController, op_long_cruise_jerk
from openpilot.starpilot.longitudinal.set_speed_assist.tests.helpers import ENGAGED, T_IDXS, FakeParams, build_sm, near, plan_ramp

V_EGO = 20.
STANDARD = log.LongitudinalPersonality.standard
A_CRUISE = 0.5  # the planner's cruise candidate
RISE = op_long_cruise_jerk(V_EGO)
SETTLE_FRAMES = round(2. / (min(c.REENGAGE_JERK, op_long_cruise_jerk(40.)) * DT)) + 10  # a 2 m/s^2 gap
FAR = 40.  # m/s under the target: the root is past every ceiling


def run(ctl, n, sm, a_model=0., a_cruise=A_CRUISE, **kwargs):
  args = {**ENGAGED, **kwargs}
  out = a_model
  for _ in range(n):
    out = ctl.update(sm, a_model, a_cruise, **args)
  return out


def new_controller(enabled=True):
  return SetSpeedAssistController(params=FakeParams(SetSpeedAssist=enabled), dt=DT)


def _plan(f):
  return np.asarray([f(t) for t in T_IDXS])


def bend_ahead(lat):
  """A bend 5-8 s out that takes lat m/s^2 at today's speed."""
  return _plan(lambda t: lat / V_EGO if 5. <= t <= 8. else 0.)


class TestController(unittest.TestCase):
  def settled(self, a_cruise=A_CRUISE, **kwargs):
    ctl = new_controller()
    run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_cruise=a_cruise, **kwargs)
    self.assertAlmostEqual(ctl.lift, ctl.floor, places=6)
    return ctl

  def test_param_key(self):
    self.assertEqual(c.PARAM, "SetSpeedAssist")

  def test_disabled_passes_model_through(self):
    ctl = new_controller(enabled=False)
    self.assertEqual(run(ctl, 200, build_sm(V_EGO), a_model=0.1), 0.1)
    self.assertEqual(ctl.inhibit, Inhibit.disabled)

  def test_idle_resets_at_once(self):
    for kwargs in ({"is_e2e": False}, {"reset_state": True}):
      with self.subTest(kwargs):
        ctl = self.settled()
        self.assertEqual(run(ctl, 1, build_sm(V_EGO), a_model=0.05, **kwargs), 0.05)
        self.assertEqual((ctl.inhibit, ctl.lift), (Inhibit.inactive, 0.))

  def test_lifts_from_the_first_frame_on_the_jerk_budget(self):
    # the gap holds still, so the lift moves only with its authority, at the re-engage budget
    ctl = new_controller()
    lifts = [run(ctl, 1, build_sm(V_EGO), a_cruise=1.5) for _ in range(SETTLE_FRAMES)]
    self.assertAlmostEqual(lifts[0], c.REENGAGE_JERK * DT, places=9)
    self.assertLessEqual(max(np.diff([0.] + lifts)), c.REENGAGE_JERK * DT + 1e-9)
    self.assertAlmostEqual(lifts[-1], 1.5, places=6)
    self.assertEqual(ctl.inhibit, Inhibit.none)

  def test_lift_never_rises_past_the_cruise_jerk(self):
    # a small gap re-engages faster, but the lift still rises no faster than the cruise candidate
    ctl = new_controller()
    lifts = [run(ctl, 1, build_sm(V_EGO), a_cruise=0.1) for _ in range(SETTLE_FRAMES)]
    self.assertLessEqual(max(np.diff([0.] + lifts)), RISE * DT + 1e-9)
    self.assertAlmostEqual(lifts[-1], 0.1, places=6)

  def test_floor_is_the_cruise_candidate(self):
    # with the model holding, the e2e candidate meets chill mode's acceleration and goes no higher
    self.assertAlmostEqual(run(new_controller(), SETTLE_FRAMES, build_sm(V_EGO), a_model=0.2), A_CRUISE, places=6)
    self.assertEqual(run(new_controller(), SETTLE_FRAMES, build_sm(V_EGO), a_model=0.7), 0.7)

  def test_never_adds_throttle_over_the_set_speed(self):
    # the cruise candidate braking: nothing to lift toward
    self.assertEqual(run(new_controller(), SETTLE_FRAMES, build_sm(V_EGO), a_model=-0.05, a_cruise=-0.3), -0.05)

  def test_lift_fades_in_above_walking_pace(self):
    for v, gain in (((c.MIN_SPEED + c.FULL_SPEED) / 2, 0.5), (c.MIN_SPEED - 1., 0.)):
      with self.subTest(v=v):
        ctl = new_controller()
        run(ctl, 1, build_sm(v), a_cruise=1.5)
        self.assertAlmostEqual(ctl.gain, gain, places=6)

  def test_leaves_stop_and_go_braking_alone(self):
    # a gentle stop under walking pace: nothing pulls the model's braking toward the cruise candidate
    self.assertEqual(run(new_controller(), SETTLE_FRAMES, build_sm(c.MIN_SPEED - 1.), a_model=-0.15), -0.15)

  def test_gain_fades_the_lift_as_the_model_slows(self):
    ctl = new_controller()
    a_model = sum(c.GAIN_BP) / 2
    run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=a_model)
    self.assertAlmostEqual(ctl.lift, 0.5 * (A_CRUISE - a_model), places=6)
    run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=c.GAIN_BP[0])
    self.assertEqual(ctl.lift, 0.)

  def test_never_lowers_the_model(self):
    # over the set speed, braking or a bend ahead: the e2e candidate is the model's or above
    for sm, a_model, a_cruise in [(build_sm(V_EGO), 0.2, -0.5), (build_sm(V_EGO), -1.5, A_CRUISE),
                                  (build_sm(V_EGO, yaw_rate=bend_ahead(2.)), 0., A_CRUISE)]:
      ctl = self.settled()
      for _ in range(200):
        self.assertGreaterEqual(run(ctl, 1, sm, a_model=a_model, a_cruise=a_cruise), a_model)
      self.assertEqual(ctl.lift, 0.)

  def test_lets_go_of_the_models_braking_at_once(self):
    ctl = self.settled(a_cruise=1.5)
    # halfway into the fade, the lift drops to its new value in one frame instead of easing off
    a_model = sum(c.GAIN_BP) / 2
    run(ctl, 1, build_sm(V_EGO), a_model=a_model, a_cruise=1.5)
    self.assertAlmostEqual(ctl.lift, 0.5 * (ctl.floor - a_model), places=6)
    ctl = self.settled(a_cruise=1.5)
    self.assertEqual(run(ctl, 1, build_sm(V_EGO), a_model=-0.5, a_cruise=1.5), -0.5)
    self.assertEqual(ctl.lift, 0.)

  def test_far_plan_flicker_does_not_move_the_lift(self):
    # the plan's far end flickering on a straight road, up to the drop band
    ctl = self.settled()
    lifts = []
    for i in range(100):
      dip = 0.99 * -c.GAIN_BP[1] * c.HORIZON if (i // 5) % 2 else 0.
      lifts.append(run(ctl, 1, build_sm(V_EGO, plan_v=plan_ramp(V_EGO, dip))))
    self.assertEqual(np.ptp(lifts), 0.)
    self.assertEqual(ctl.plan_fade, 1.)

  def test_plan_drop_fades_the_lift(self):
    # a gentle coast (inside the gain's band) toward a stop or a light: the lift lets it coast
    for drop, plan_fade in ((0., 1.), (1., 1.), (2., 0.5), (3., 0.), (8., 0.)):
      with self.subTest(drop=drop):
        ctl = new_controller()
        a_model = -0.05
        run(ctl, SETTLE_FRAMES, build_sm(V_EGO, plan_v=plan_ramp(V_EGO, drop)), a_model=a_model)
        self.assertAlmostEqual(ctl.plan_fade, plan_fade, places=6)
        self.assertAlmostEqual(ctl.gain, plan_fade, places=6)
        self.assertAlmostEqual(ctl.lift, plan_fade * (A_CRUISE - a_model), places=6)

  def test_coast_toward_a_stop_lets_go_of_the_lift(self):
    # lifted on an open road, then the plan starts ending at a stop: the lift goes, never below the model
    ctl = self.settled()
    drops = np.linspace(0., 6., 60)
    out = [run(ctl, 1, build_sm(V_EGO, plan_v=plan_ramp(V_EGO, d)), a_model=-0.05) for d in drops]
    self.assertGreaterEqual(min(out), -0.05)
    self.assertTrue(all(o == -0.05 for o, d in zip(out, drops, strict=True) if d >= c.HORIZON * -c.GAIN_BP[0]))
    self.assertEqual(ctl.lift, 0.)

  def test_closing_lead_fades_the_lift(self):
    for lead, plan_fade in (
      ((60., 0.), 1.),     # steady lead
      ((60., -0.9), 1.),   # barely closing
      ((30., -3.), 0.),    # closing hard, 10 s to contact
      ((60., -3.), 0.75),  # closing hard, 20 s to contact
      ((150., -3.), 1.),   # 50 s to contact
      ((20., -2.), 0.5),   # closing gently, 10 s to contact
      ((60., 2.), 1.),     # pulling away
    ):
      with self.subTest(lead=lead):
        ctl = new_controller()
        run(ctl, SETTLE_FRAMES, build_sm(V_EGO, lead=lead))
        self.assertAlmostEqual(ctl.plan_fade, plan_fade, places=6)
        self.assertAlmostEqual(ctl.lift, plan_fade * A_CRUISE, places=6)

  def test_lead_gate_fades_the_lift_for_a_braking_lead(self):
    # Accel Boost's lead gate (Bosch-A Honda): a braking lead inside 4 s headway takes the lift out
    # over 0.5 s, and it comes back over 2 s once the lead stops braking. Without the gate, no change.
    def lead_sm(a_lead):
      sm = build_sm(V_EGO)
      rs = messaging.new_message('radarState')
      rs.radarState.leadOne.present = True
      rs.radarState.leadOne.dRel = 40.  # 2 s headway at V_EGO
      rs.radarState.leadOne.vRel = 0.   # not closing: the plan fade stays 1
      rs.radarState.leadOne.aLeadK = a_lead
      return {**sm, 'radarState': rs.radarState.as_reader()}
    braking, steady = lead_sm(-1.), lead_sm(0.)
    out_s = c.LEAD_GATE_FADE_OUT_S + 0.5  # the fade, then the lift's own fall at LIFT_FALL
    for gate in (False, True):
      with self.subTest(lead_gate=gate):
        ctl = self.settled(lead_gate=gate)
        out = [run(ctl, 1, braking, lead_gate=gate) for _ in range(round(out_s / DT))]
        self.assertGreaterEqual(min(out), 0.)
        if gate:
          self.assertEqual(ctl.lead_scale, 0.)
          self.assertEqual(ctl.lift, 0.)
          run(ctl, round(c.LEAD_GATE_FADE_IN_S / DT) + SETTLE_FRAMES, steady, lead_gate=gate)
          self.assertEqual(ctl.lead_scale, 1.)
          self.assertAlmostEqual(ctl.lift, A_CRUISE, places=6)
        else:
          self.assertEqual(ctl.lead_scale, 1.)
          self.assertAlmostEqual(ctl.lift, A_CRUISE, places=6)

  def test_closing_lead_fade_is_smooth(self):
    ctl = new_controller()
    fades = []
    for d in np.linspace(120., 20., 101):  # closing at 3 m/s from 40 s to 7 s out
      run(ctl, 1, build_sm(V_EGO, lead=(d, -3.)))
      fades.append(ctl.plan_fade)
    self.assertEqual((fades[0], fades[-1]), (1., 0.))
    self.assertLessEqual(max(np.abs(np.diff(fades))), 0.05 + 1e-9)  # 1 m of 100 per step

  def test_a_bend_ahead_fades_the_lift(self):
    for lat, fade in ((0.4, 1.), (0.8, 0.5), (1.2, 0.)):
      with self.subTest(lat=lat):
        ctl = new_controller()
        run(ctl, SETTLE_FRAMES, build_sm(V_EGO, yaw_rate=bend_ahead(lat)))
        self.assertAlmostEqual(ctl.fade, fade, places=6)
        self.assertAlmostEqual(ctl.lift, fade * A_CRUISE, places=6)

  def test_bend_flicker_moves_the_lift_on_the_jerk_budget(self):
    # the path's bend over 10 s flickering frame to frame across the fade's band: the lift follows on
    # the fade's budget, not at the frame rate
    for a_cruise in (0.5, 1.5):
      with self.subTest(a_cruise=a_cruise):
        ctl = self.settled(a_cruise=a_cruise)
        lifts = [ctl.lift]
        for i in range(200):
          lat = 0.9 if i % 2 else 0.7
          lifts.append(run(ctl, 1, build_sm(V_EGO, yaw_rate=bend_ahead(lat)), a_cruise=a_cruise))
        self.assertLessEqual(max(np.abs(np.diff(lifts))), c.FADE_JERK * DT + 1e-9)
        self.assertTrue(0.25 * a_cruise - 1e-9 <= ctl.lift <= 0.75 * a_cruise + 1e-9)  # inside the flicker's fades

  def test_bend_ahead_eases_the_lift_off_on_the_jerk_budget(self):
    ctl = self.settled(a_cruise=1.5)
    lifts = [run(ctl, 1, build_sm(V_EGO, yaw_rate=bend_ahead(1.2)), a_cruise=1.5) for _ in range(SETTLE_FRAMES)]
    self.assertAlmostEqual(lifts[0], 1.5 - c.FADE_JERK * DT, places=6)
    self.assertEqual(lifts[-1], 0.)

  def test_lane_change_drops_the_lift(self):
    ctl = self.settled()
    run(ctl, 1, build_sm(V_EGO, lane_change=True))
    self.assertEqual(ctl.inhibit, Inhibit.laneChange)
    self.assertAlmostEqual(ctl.lift, ctl.floor - c.LIFT_FALL * DT, places=6)

  def test_invalid_model_passes_through(self):
    ctl = self.settled()
    sm = build_sm(V_EGO, plan_v=_plan(lambda t: float('nan') if t > 5. else V_EGO))
    self.assertEqual(run(ctl, 1, sm, a_model=0.05), 0.05)
    self.assertEqual((ctl.inhibit, ctl.lift), (Inhibit.invalid, 0.))

    md = messaging.new_message('modelV2')
    md.modelV2.velocity.x = [V_EGO] * 10
    md.modelV2.orientationRate.z = [0.] * 10
    sm = {**build_sm(V_EGO), 'modelV2': md.modelV2.as_reader()}
    self.assertEqual(run(ctl, 1, sm, a_model=0.05), 0.05)
    self.assertEqual(ctl.inhibit, Inhibit.invalid)

    self.assertEqual(run(ctl, 1, build_sm(float('nan')), a_model=0.05), 0.05)
    self.assertEqual(ctl.inhibit, Inhibit.invalid)


class TestParamRead(unittest.TestCase):
  def test_reads_every_period_and_not_every_frame(self):
    params = FakeParams(SetSpeedAssist=False)
    ctl = SetSpeedAssistController(params=params, dt=DT)
    self.assertEqual(ctl.params_frames, round(1. / DT))
    run(ctl, 1, build_sm(V_EGO))
    self.assertFalse(ctl.enabled)
    params.values['SetSpeedAssist'] = True
    run(ctl, ctl.params_frames - 1, build_sm(V_EGO))
    self.assertFalse(ctl.enabled)
    run(ctl, 1, build_sm(V_EGO))
    self.assertTrue(ctl.enabled)

  def test_a_params_library_without_the_key_reads_off(self):
    class Unknown:
      def get_bool(self, key):
        raise RuntimeError("UnknownKeyName")
    ctl = SetSpeedAssistController(params=Unknown(), dt=DT)
    self.assertEqual(run(ctl, 5, build_sm(V_EGO), a_model=0.1), 0.1)
    self.assertFalse(ctl.enabled)
    self.assertFalse(ctl.engaged)

  def test_poll_then_update_is_one_frame(self):
    ctl = new_controller()
    self.assertTrue(ctl.poll())
    ctl.update(build_sm(V_EGO), 0., A_CRUISE, **ENGAGED)
    self.assertEqual(ctl.frame, 0)
    ctl.update(build_sm(V_EGO), 0., A_CRUISE, **ENGAGED)
    self.assertEqual(ctl.frame, 1)


class TestApproach(unittest.TestCase):
  def test_approach_closes_small_gaps_gently(self):
    # the floor's pace, up to the speed's ceiling
    for v, ceiling in zip(approach.APPROACH_BP, approach.APPROACH_V, strict=True):
      self.assertAlmostEqual(approach.approach_accel(v, v + FAR, STANDARD), ceiling, places=6)
    self.assertLess(approach.approach_accel(V_EGO, V_EGO + 1., STANDARD), 0.42)  # a CX-5 5>4 kickdown at highway speed

  def test_approach_is_continuous_at_the_target(self):
    # no step where the car reaches the target: the pace falls smoothly through 0
    step = 1e-3
    gaps = np.linspace(-1., 10., 11001)
    for v in (5., 15., 25.):
      with self.subTest(v=v):
        paces = np.array([approach.approach_accel(v - g, v, STANDARD) for g in gaps])
        self.assertTrue(all(np.diff(paces) >= 0.))
        self.assertLessEqual(max(np.diff(paces)), np.sqrt(2 * approach.APPROACH_JERK * step) + 1e-9)
        self.assertEqual(approach.approach_accel(v, v, STANDARD), 0.)

  def test_approach_settles_on_the_gap_near_the_target(self):
    # inside 2 * jerk the pace is the gap at 1 /s, under the root, so the floor eases in by itself
    for gap in (0.01, 0.1):
      for v in approach.APPROACH_BP:
        self.assertAlmostEqual(approach.approach_accel(v - gap, v, STANDARD), gap, places=6)
    self.assertAlmostEqual(approach.approach_accel(V_EGO - 0.2, V_EGO, STANDARD), np.sqrt(2 * approach.APPROACH_JERK * 0.2), places=6)

  def test_approach_pulls_back_over_the_target(self):
    # over the target the pace is the gap, pulling back toward it however the cruise candidate lags
    for over in (0., 0.5, 5.):
      for personality in (log.LongitudinalPersonality.relaxed, STANDARD, log.LongitudinalPersonality.aggressive):
        for v in approach.APPROACH_BP:
          self.assertAlmostEqual(approach.approach_accel(v + over, v, personality), -over, places=6)

  def test_approach_follows_the_personality(self):
    # Standard is the ceiling; Relaxed and Aggressive sit a 0.15 step either side of it at a full
    # shortfall. Read from a message as the planner does.
    def personality(name):
      msg = messaging.new_message('selfdriveState')
      msg.selfdriveState.personality = name
      return msg.as_reader().selfdriveState.personality.raw
    relaxed_p, standard_p, aggressive_p = (personality(n) for n in ('relaxed', 'standard', 'aggressive'))
    for v in approach.APPROACH_BP:
      relaxed, standard, aggressive = (approach.approach_accel(v, v + FAR, p) for p in (relaxed_p, standard_p, aggressive_p))
      self.assertAlmostEqual(relaxed, standard - 0.15, places=6)
      self.assertAlmostEqual(aggressive, standard + 0.15, places=6)

  def test_jerk_table_is_the_planners(self):
    from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MAX_BP, J_CRUISE_VALS
    self.assertEqual(c._OP_LONG_J_BP, A_CRUISE_MAX_BP)
    self.assertEqual(c._OP_LONG_J_VALS, J_CRUISE_VALS)
    self.assertTrue(near(op_long_cruise_jerk(5.), 1.4))


if __name__ == "__main__":
  unittest.main()
