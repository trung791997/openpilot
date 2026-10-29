"""Static tests for bosch_a_stop_gap (stop hold + no gas inside the stop gap). Not road evidence."""
from types import SimpleNamespace
import unittest

from openpilot.starpilot.longitudinal import bosch_a_stop_gap as sg

BOSCH_A = SimpleNamespace(brand="honda", carFingerprint="HONDA_CIVIC_BOSCH", openpilotLongitudinalControl=True,
                          radarUnavailable=False)
NO_RADAR = SimpleNamespace(brand="honda", carFingerprint="HONDA_CIVIC_BOSCH", openpilotLongitudinalControl=True,
                           radarUnavailable=True)
NONE = SimpleNamespace(present=False, radar=False, dRel=0.0, vLead=0.0)


def lead(d, v, radar=True):
  return SimpleNamespace(present=True, radar=radar, dRel=d, vLead=v)


def step(gap, l1, target, should_stop, v_ego, standstill=False, l2=NONE, reset_state=False):
  return gap.step(lead_one=l1, lead_two=l2, target=target, should_stop=should_stop, v_ego=v_ego, standstill=standstill,
                  reset_state=reset_state)


class TestStopHold(unittest.TestCase):
  def test_route46_flicker_keeps_the_stop(self):
    # 46 15:37.92: stopped 4.11 m behind the stopped Audi, the model's shouldStop drops for two frames
    gap = sg.BoschAStopGap(BOSCH_A)
    self.assertEqual(step(gap, lead(4.11, 0.2), -0.4, True, 0.0, standstill=True), (-0.4, True))
    for target in (-0.10, -0.06):
      out, stop = step(gap, lead(4.11, 0.42), target, False, 0.0, standstill=True)
      self.assertTrue(stop)
      self.assertLessEqual(out, sg.NO_GAS_ACCEL)

  def test_lead_pulling_off_releases(self):
    gap = sg.BoschAStopGap(BOSCH_A)
    step(gap, lead(4.5, 0.0), -0.3, True, 0.0, standstill=True)
    self.assertTrue(gap.holding)
    self.assertEqual(step(gap, lead(4.6, sg.RELEASE_LEAD_V), 0.4, False, 0.0, standstill=True), (0.4, False))
    self.assertFalse(gap.holding)

  def test_lost_lead_and_disengage_release(self):
    for kwargs in ({'l1': NONE}, {'l1': lead(4.5, 0.0), 'reset_state': True}, {'l1': lead(4.5, 0.0, radar=False)}):
      gap = sg.BoschAStopGap(BOSCH_A)
      step(gap, lead(4.5, 0.0), -0.3, True, 0.0, standstill=True)
      self.assertEqual(step(gap, target=0.3, should_stop=False, v_ego=0.0, standstill=True, **kwargs), (0.3, False))

  def test_no_hold_behind_a_far_or_moving_lead(self):
    gap = sg.BoschAStopGap(BOSCH_A)
    self.assertEqual(step(gap, lead(sg.HOLD_D_M + 0.1, 0.0), 0.2, False, 0.0, standstill=True), (0.2, False))
    self.assertEqual(step(gap, lead(4.0, 1.0), 0.2, False, 0.0, standstill=True), (0.2, False))
    self.assertFalse(gap.holding)

  def test_not_a_bosch_a_car_with_radar(self):
    gap = sg.BoschAStopGap(NO_RADAR)
    self.assertEqual(step(gap, lead(4.0, 0.0), 0.2, False, 0.0, standstill=True), (0.2, False))


class TestCrawlCap(unittest.TestCase):
  def test_route46_crawl_gets_no_gas(self):
    # 46 15:52.06: 0.68 m/s, 3.67 m behind the Audi rolling at 0.69, target +0.12
    gap = sg.BoschAStopGap(BOSCH_A)
    self.assertEqual(step(gap, lead(3.67, 0.69), 0.12, False, 0.68), (sg.NO_GAS_ACCEL, False))

  def test_harder_brake_passes(self):
    gap = sg.BoschAStopGap(BOSCH_A)
    self.assertEqual(step(gap, lead(3.0, 0.2), -1.0, False, 1.0), (-1.0, False))

  def test_lead_pulling_away_gets_gas(self):
    # 46 15:54.30: 0.39 m/s, the Audi at 0.95 and the gap growing
    gap = sg.BoschAStopGap(BOSCH_A)
    self.assertEqual(step(gap, lead(2.98, 0.95), 0.11, False, 0.39), (0.11, False))

  def test_outside_the_gap_or_faster_untouched(self):
    gap = sg.BoschAStopGap(BOSCH_A)
    self.assertEqual(step(gap, lead(sg.CRAWL_D_M + 0.1, 0.5), 0.3, False, 1.0), (0.3, False))
    self.assertEqual(step(gap, lead(5.5, 1.0), 0.3, False, 1.0), (0.3, False))  # matched speed past CRAWL_CLOSE_D_M
    self.assertEqual(step(gap, lead(5.5, 0.7), 0.3, False, 1.0), (sg.NO_GAS_ACCEL, False))  # closing
    self.assertEqual(step(gap, lead(5.0, 2.0), 0.3, False, sg.CRAWL_V_MAX), (0.3, False))

  def test_closest_radar_lead_is_used(self):
    gap = sg.BoschAStopGap(BOSCH_A)
    self.assertEqual(step(gap, lead(20.0, 5.0), 0.3, False, 1.0, l2=lead(3.5, 0.5)), (sg.NO_GAS_ACCEL, False))


class TestConstants(unittest.TestCase):
  def test_no_gas_is_under_the_bosch_gas_floor(self):
    from opendbc.car.honda.values import CarControllerParams
    self.assertLess(sg.NO_GAS_ACCEL, CarControllerParams.BOSCH_GAS_LOOKUP_BP[0])

  def test_release_speed_matches_breakaway(self):
    from openpilot.starpilot.longitudinal.lead_takeoff import BREAKAWAY_MIN_LEAD_SPEED
    self.assertEqual(sg.RELEASE_LEAD_V, BREAKAWAY_MIN_LEAD_SPEED)


if __name__ == "__main__":
  unittest.main()
