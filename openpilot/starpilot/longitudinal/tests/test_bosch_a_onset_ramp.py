"""Static tests for the brake onset ramp inside HondaBoschALongEnhancement.step (stock-like brake onset for far leads).
Not road evidence."""
import random
from types import SimpleNamespace
import unittest

from openpilot.starpilot.longitudinal import honda_bosch_a_long as hbl
from openpilot.starpilot.longitudinal.honda_bosch_a_long import HondaBoschALongEnhancement

BOSCH_A = SimpleNamespace(brand="honda", carFingerprint="HONDA_CIVIC_BOSCH", openpilotLongitudinalControl=True,
                          radarUnavailable=False)
OTHER = SimpleNamespace(brand="toyota", carFingerprint="TOYOTA_RAV4", openpilotLongitudinalControl=True,
                        radarUnavailable=False)
NONE = SimpleNamespace(present=False, dRel=0.0, vRel=0.0, aLeadK=0.0, vLead=0.0)
DT = 0.05
V_EGO = 20.0
STEP = hbl.ONSET_JERK * DT


def lead(d, v_rel, a=-3.0):
  """A present lead. a=-3 is hard enough (with TTC <= 10 s) to latch the law's emergency pass-through, which is the only
  place the onset ramp can bind now that the law is always on."""
  return SimpleNamespace(present=True, dRel=d, vRel=v_rel, aLeadK=a, vLead=max(V_EGO + v_rel, 0.0))


def make(cp=BOSCH_A):
  return HondaBoschALongEnhancement(cp, DT)


def run(enh, l1, targets, prev=0.0, fcw=False, stopping=False, l2=NONE):
  out = []
  for target in targets:
    prev = enh.step(lead_one=l1, lead_two=l2, prev=prev, target=target, v_ego=V_EGO, stopping=stopping, standstill=False,
                    reset_state=False, fcw=fcw)
    out.append(prev)
  return out


class TestOnsetRamp(unittest.TestCase):
  def test_far_brake_builds_at_onset_jerk_and_reaches_the_peak(self):
    # 84 E1 shape: lead far, closing fast, target steps 0 -> -3.5 in one frame (TTC 9.5 s so the emergency latch holds)
    out = run(make(), lead(70.0, -7.4), [-3.5] * 12)
    self.assertAlmostEqual(out[0], -STEP)
    for a, b in zip(out, out[1:], strict=False):
      self.assertGreaterEqual(b, a - STEP - 1e-9)
    self.assertEqual(out[-1], -3.5)  # the peak is never reduced

  def test_close_lead_at_onset_is_exempt_for_the_episode(self):
    # 2f5 11830: onset with the lead at 38.1 m
    enh = make()
    self.assertEqual(run(enh, lead(38.1, -5.0), [-3.5]), [-3.5])
    self.assertTrue(enh.onset_free)
    # still exempt while the episode lasts, even after the lead reads farther
    self.assertEqual(run(enh, lead(45.0, -5.0), [-2.0, -3.0], prev=-3.5), [-2.0, -3.0])

  def test_episode_decision_is_latched_until_the_brake_ends(self):
    enh = make()
    run(enh, lead(80.0, -10.0), [-1.0])
    self.assertFalse(enh.onset_free)
    # closing to 39 m mid-brake does not exempt it (R5 replay: no gain); TTC 39/3 = 13 s, emergency still latched
    self.assertAlmostEqual(run(enh, lead(39.0, -3.0), [-3.0], prev=-0.4)[0], -0.4 - STEP)
    # REWRITTEN: the episode ends when the output AND last cycle's published target are back at or above END_ACCEL.
    # The old ramp ended on its own input the same cycle; this is the same event seen one cycle later at most.
    run(enh, lead(39.0, -3.0), [0.0], prev=-1.0)
    self.assertTrue(enh.onset_episode)
    run(enh, lead(39.0, -3.0), [0.0], prev=0.0)
    self.assertFalse(enh.onset_episode)
    self.assertFalse(enh.onset_free)
    # a new brake starts with the lead close: exempt
    self.assertEqual(run(enh, lead(39.0, -3.0), [-2.0])[0], -2.0)

  def test_low_ttc_stands_down_for_the_rest_of_the_episode(self):
    enh = make()
    run(enh, lead(60.0, -10.0), [-3.5])
    self.assertEqual(run(enh, lead(50.0, -25.0), [-3.5], prev=-0.4), [-3.5])  # TTC 2.0 s
    self.assertEqual(run(enh, lead(60.0, -5.0), [-3.5], prev=-3.5), [-3.5])  # stays down

  def test_railed_vrel_only_shortens_ttc(self):
    # U11 low rail vRel -12 at 22 m/s... the TTC check reads it as published, which can only stand the ramp down
    self.assertEqual(run(make(), lead(23.0, -12.0), [-3.5]), [-3.5])  # also under the 40 m exemption
    enh = make()
    run(enh, lead(80.0, -5.0), [-1.0])
    self.assertEqual(run(enh, lead(24.0, -12.0), [-3.5], prev=-0.4), [-3.5])

  def test_fcw_and_stopping_pass_through(self):
    self.assertEqual(run(make(), lead(60.0, -8.0), [-3.5], fcw=True), [-3.5])
    self.assertEqual(run(make(), lead(60.0, -8.0), [-3.5], stopping=True), [-3.5])

  def test_second_lead_close_exempts(self):
    out = run(make(), lead(80.0, -9.0), [-3.5], l2=lead(20.0, 0.0, a=0.0))
    self.assertEqual(out, [-3.5])

  def test_no_lead_brake_is_limited_by_the_law_not_the_ramp(self):
    # REWRITTEN: with no lead the law (jerk outside, 5 m/s^3) is tighter than the ramp (8) and binds first, so the output
    # equals stock_feel_target and the ramp never changes it (the old test ran the ramp alone).
    enh = make()
    out = run(enh, NONE, [-2.0, -2.0, -2.0])
    self.assertAlmostEqual(out[0], -hbl.STOCK_FEEL_JERK_OUTSIDE * DT)
    self.assertAlmostEqual(out[1], -2 * hbl.STOCK_FEEL_JERK_OUTSIDE * DT)
    self.assertTrue(enh.onset_episode)
    self.assertLess(hbl.STOCK_FEEL_JERK_OUTSIDE, hbl.ONSET_JERK)

  def test_gentle_target_and_rises_pass_through(self):
    # REWRITTEN: no lead, so the law's jerk limit (0.25 per frame) does not touch a -0.25 target.
    enh = make()
    self.assertEqual(run(enh, NONE, [-0.25, 0.5, 1.0]), [-0.25, 0.5, 1.0])
    self.assertFalse(enh.onset_episode)
    # the ramp starts from min(prev, 0): a throttle cut to zero is immediate
    self.assertAlmostEqual(run(make(), lead(70.0, -10.0), [-1.0], prev=1.2)[0], -STEP)

  def test_reset_and_other_cars_pass_through(self):
    enh = make()
    run(enh, lead(80.0, -10.0), [-1.0])
    self.assertTrue(enh.onset_episode)
    self.assertEqual(enh.step(lead_one=lead(80.0, -10.0), lead_two=NONE, prev=-0.4, target=-3.5, v_ego=V_EGO,
                              stopping=False, standstill=False, reset_state=True, fcw=False), -3.5)
    self.assertFalse(enh.onset_episode)
    self.assertFalse(enh.onset_free)
    self.assertEqual(run(make(OTHER), lead(80.0, -10.0), [-3.5]), [-3.5])

  def test_standstill_clears_the_episode(self):
    enh = make()
    run(enh, lead(80.0, -10.0), [-1.0])
    self.assertEqual(enh.step(lead_one=NONE, lead_two=NONE, prev=-0.4, target=-3.5, v_ego=0.0, stopping=False,
                              standstill=True, reset_state=False, fcw=False), -3.5)
    self.assertFalse(enh.onset_episode)

  # ---- new: the ramp inside the emergency pass-through, and only there ----
  def test_emergency_latch_far_lead_deepens_at_most_onset_jerk(self):
    # (a) lead at 60 m, closing 12 m/s (TTC 5 s), braking -3: emergency latched, the law passes -3.5 through
    out = run(make(), lead(60.0, -12.0), [-3.5] * 20)
    self.assertAlmostEqual(out[0], -STEP)
    prev = 0.0
    for o in out:
      self.assertGreaterEqual(o, prev - STEP - 1e-9)
      prev = o
    self.assertEqual(out[-1], -3.5)

  def test_emergency_latch_close_lead_at_episode_start_passes_through(self):
    # (b) same braking lead at 35 m when the episode starts: exempt, no ramp
    enh = make()
    self.assertEqual(run(enh, lead(35.0, -7.0), [-3.5] * 3), [-3.5] * 3)
    self.assertTrue(enh.onset_free)

  def test_ttc_falling_to_two_seconds_frees_the_rest_of_the_episode(self):
    # (c)
    enh = make()
    self.assertAlmostEqual(run(enh, lead(60.0, -12.0), [-3.5, -3.5], prev=0.0)[1], -2 * STEP)
    self.assertFalse(enh.onset_free)
    self.assertEqual(run(enh, lead(40.0, -20.0), [-3.5], prev=-2 * STEP), [-3.5])  # TTC 2.0 s
    self.assertTrue(enh.onset_free)
    # TTC recovers, the target lets off to -2 and deepens again: still free for the rest of the episode
    self.assertEqual(run(enh, lead(60.0, -12.0), [-2.0, -3.5], prev=-3.5), [-2.0, -3.5])

  def test_non_emergency_law_output_is_never_changed_by_the_ramp(self):
    # (d) seeded sweep, fresh state per sample: step() == stock_feel_target() whenever the emergency latch is off
    rng = random.Random(20261111)
    checked = binding = 0
    for _ in range(20000):
      leads = []
      for _ in range(2):
        if rng.random() < 0.25:
          leads.append(NONE)
          continue
        d = rng.uniform(3.0, 150.0)
        v_rel = rng.uniform(-18.0, 6.0)
        leads.append(lead(d, v_rel, a=rng.uniform(-4.0, 1.5)))
      v_ego = rng.uniform(0.0, 32.0)
      prev = rng.uniform(-3.5, 1.5)
      target = rng.uniform(-3.5, 1.5)
      if hbl.stock_feel_emergency(tuple(leads), v_ego, False):
        continue
      expected = hbl.stock_feel_target(tuple(leads), prev, target, DT, v_ego, False, False)
      enh = make()
      got = enh.step(lead_one=leads[0], lead_two=leads[1], prev=prev, target=target, v_ego=v_ego, stopping=False,
                     standstill=False, reset_state=False, fcw=False)
      self.assertEqual(got, expected, (leads, v_ego, prev, target))
      checked += 1
      binding += expected < hbl.ONSET_START_ACCEL
    self.assertGreater(checked, 5000)
    self.assertGreater(binding, 500)  # the sweep does start brake episodes, so the ramp was in play


if __name__ == "__main__":
  unittest.main()
