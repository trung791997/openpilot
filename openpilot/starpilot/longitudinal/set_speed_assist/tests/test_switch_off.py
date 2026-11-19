"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ported from zoompilot (e2e_set_speed tests, commits e6baf0b50 and 2e9506a87) as unittest, see ../NOTICE.md.

Switched off with a lift on, the lift goes out at its own rate instead of stepping off the
accel request.
"""
import unittest

from openpilot.common.realtime import DT_MDL as DT
from openpilot.starpilot.longitudinal.set_speed_assist import controller as c
from openpilot.starpilot.longitudinal.set_speed_assist.controller import Inhibit
from openpilot.starpilot.longitudinal.set_speed_assist.tests.helpers import build_sm, near
from openpilot.starpilot.longitudinal.set_speed_assist.tests.test_controller import SETTLE_FRAMES, V_EGO, new_controller, run


def settled():
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO))
  return ctl


class TestSwitchOff(unittest.TestCase):
  def test_switch_off_lets_the_lift_out_at_lift_fall(self):
    ctl = settled()
    lift = ctl.lift
    self.assertGreater(lift, 0.)
    ctl.params.values[c.PARAM] = False
    ctl.enabled = False  # the params pass lands within PARAMS_UPDATE_PERIOD
    sm = build_sm(V_EGO)
    out = run(ctl, 1, sm)
    self.assertTrue(near(out, lift - c.LIFT_FALL * DT))
    self.assertEqual(ctl.inhibit, Inhibit.disabled)
    run(ctl, round(lift / (c.LIFT_FALL * DT)) + 2, sm)
    self.assertEqual(ctl.lift, 0.)
    self.assertEqual(run(ctl, 1, sm, a_model=0.3), 0.3)  # then idles on the model

  def test_switch_off_while_idle_is_immediate(self):
    ctl = settled()
    ctl.enabled = False
    ctl.lift = 0.
    self.assertEqual(run(ctl, 1, build_sm(V_EGO), a_model=0.2), 0.2)
    self.assertEqual(ctl.inhibit, Inhibit.disabled)

  def test_engaged_follows_the_lift_out(self):
    # the planner hook keeps Accel Boost off until the lift is gone
    ctl = settled()
    ctl.params.values[c.PARAM] = False
    ctl.enabled = False
    self.assertTrue(ctl.engaged)
    run(ctl, round(ctl.lift / (c.LIFT_FALL * DT)) + 2, build_sm(V_EGO))
    self.assertFalse(ctl.engaged)


if __name__ == "__main__":
  unittest.main()
