"""Tests for the far-lead brake limit (TEST feature, default OFF).

The limit bounds braking demanded for a lead that is far away in BOTH time and distance. Its
whole evidence base is ONE observed fault, so the tests that matter most here are the negative
ones: every correct hard brake in the 16-segment corpus must pass through untouched, and the
limit must be inert unless explicitly enabled.

Geometry for each case is taken from the corpus, so a change that breaks one of these is
breaking against recorded road data rather than an invented scenario (D-009).
"""

from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  FAR_LEAD_BRAKE_LIMIT_ACCEL,
  FAR_LEAD_BRAKE_LIMIT_FULL_TTC,
  FAR_LEAD_BRAKE_LIMIT_MIN_DIST,
  FAR_LEAD_BRAKE_LIMIT_MIN_HEADWAY,
  FAR_LEAD_BRAKE_LIMIT_MIN_SPEED,
  FAR_LEAD_BRAKE_LIMIT_MIN_TTC,
  LongitudinalPlanner,
)

ACCEL_MIN = -3.5


def lead(dRel, vRel, status=True, radar=True):
  return SimpleNamespace(status=status, dRel=dRel, vRel=vRel, radar=radar,
                         vLead=0.0, aLeadK=0.0, yRel=0.0, modelProb=1.0)


class _Planner:
  """Bare instance: the limit is a pure function of the lead geometry."""
  get_far_lead_brake_limit = LongitudinalPlanner.get_far_lead_brake_limit


@pytest.fixture
def planner():
  return _Planner()


# (label, dRel, vRel, vEgo, commanded accel) straight from the corpus event table.
CORRECT_BRAKES = [
  ("3053f5a6  TTC 5.0",   61.3, -12.14, 19.16, -3.50),
  ("4b66cbf6  TTC 5.4",   72.4, -13.28, 11.52, -3.25),
  ("f66399a5  TTC 5.3",   18.5,  -3.53,  4.53, -2.40),
  ("f66399a5  hdwy 2.46", 33.9,  -0.62, 13.76, -2.17),
  ("97566dde  hdwy 2.04", 40.0,  -2.20, 19.55, -2.00),
]


class TestNegativeControls:
  """Every correct brake in the corpus must be left alone. This is the point of the feature."""

  @pytest.mark.parametrize("label,d,v,ve,accel", CORRECT_BRAKES,
                           ids=[c[0] for c in CORRECT_BRAKES])
  def test_correct_corpus_brakes_are_untouched(self, planner, label, d, v, ve, accel):
    limit = planner.get_far_lead_brake_limit(lead(d, v), ve, ACCEL_MIN)
    assert limit is None or accel >= limit, (
      f"{label}: a correct {accel:.2f} m/s2 brake would be capped to {limit}"
    )

  def test_vision_lead_is_never_limited(self, planner):
    """Vision leads have their own caps in this file; this must not double-govern them."""
    assert planner.get_far_lead_brake_limit(lead(62.4, -4.36, radar=False), 17.55, ACCEL_MIN) is None

  def test_no_lead_is_never_limited(self, planner):
    assert planner.get_far_lead_brake_limit(None, 20.0, ACCEL_MIN) is None
    assert planner.get_far_lead_brake_limit(lead(62.4, -4.36, status=False), 17.55, ACCEL_MIN) is None

  def test_inert_below_the_speed_floor(self, planner):
    """Stop-and-go has its own behaviour and is out of scope."""
    slow = FAR_LEAD_BRAKE_LIMIT_MIN_SPEED - 1.0
    assert planner.get_far_lead_brake_limit(lead(80.0, -0.5), slow, ACCEL_MIN) is None

  def test_inert_inside_the_headway_floor(self, planner):
    ve = 20.0
    close = (FAR_LEAD_BRAKE_LIMIT_MIN_HEADWAY - 0.5) * ve
    assert planner.get_far_lead_brake_limit(lead(close, -0.5), ve, ACCEL_MIN) is None

  def test_inert_inside_the_distance_floor(self, planner):
    """Headway alone is not enough: dRel / v_ego RISES as the car slows, so an ordinary
    deceleration satisfies it on its way down. Geometry from 0000022e--2c6875ff90 seg 8, a
    correct stop-and-go brake this limit fired on before the distance gate existed."""
    limit = planner.get_far_lead_brake_limit(lead(27.2, -2.17), 8.0, ACCEL_MIN)
    assert 27.2 / 8.0 >= FAR_LEAD_BRAKE_LIMIT_MIN_HEADWAY, "the headway gate does NOT catch this"
    assert 27.2 < FAR_LEAD_BRAKE_LIMIT_MIN_DIST, "this geometry no longer exercises the distance gate"
    assert limit is None, "a 27 m lead is not a far lead, whatever the headway says"

  def test_the_distance_gate_is_where_the_constant_says(self, planner):
    """Pins the gate to FAR_LEAD_BRAKE_LIMIT_MIN_DIST rather than to the 27 m sample above, so
    that lowering the constant toward the correct-brake geometry fails here first. Everything
    except dRel is held far enough out to be inert on its own."""
    ve, ttc = 12.0, FAR_LEAD_BRAKE_LIMIT_FULL_TTC + 2.0
    below = FAR_LEAD_BRAKE_LIMIT_MIN_DIST - 0.5
    above = FAR_LEAD_BRAKE_LIMIT_MIN_DIST + 0.5
    assert below / ve >= FAR_LEAD_BRAKE_LIMIT_MIN_HEADWAY, "headway must not be the gate under test"
    assert planner.get_far_lead_brake_limit(lead(below, -(below / ttc)), ve, ACCEL_MIN) is None
    assert planner.get_far_lead_brake_limit(lead(above, -(above / ttc)), ve, ACCEL_MIN) is not None

  def test_real_stop_and_go_approach_is_untouched(self, planner):
    """The same event sampled across its approach: radar and vision agreed throughout and the
    lead was genuinely decelerating. None of it may be limited."""
    for d, v, ve in [(27.2, -2.17, 8.0), (26.9, -1.92, 8.0), (26.7, -1.95, 8.0),
                     (25.5, -1.84, 7.5), (21.7, -3.50, 5.4)]:
      assert planner.get_far_lead_brake_limit(lead(d, v), ve, ACCEL_MIN) is None, \
        f"dRel={d} vRel={v} vEgo={ve} is a close lead, not a far one"

  def test_inert_inside_the_ttc_floor(self, planner):
    """A lead 4 s away in headway but closing fast is urgent; the limit must stand aside."""
    ve = 20.0
    d = 80.0
    v_rel = -(d / (FAR_LEAD_BRAKE_LIMIT_MIN_TTC - 2.0))
    assert planner.get_far_lead_brake_limit(lead(d, v_rel), ve, ACCEL_MIN) is None


class TestTheFault:
  """The one positive example: 00000231--5782493b00 at t=11.21."""

  def test_the_observed_fault_is_limited(self, planner):
    limit = planner.get_far_lead_brake_limit(lead(62.4, -4.36), 17.55, ACCEL_MIN)
    assert limit is not None, "the fault geometry must be inside the limited regime"
    assert limit > -2.89, "the limit must be gentler than the -2.89 that was commanded"
    assert limit == pytest.approx(FAR_LEAD_BRAKE_LIMIT_ACCEL, abs=0.01), (
      "TTC 14.3 is past FULL_TTC so the ramp should be saturated"
    )

  def test_limit_never_authorises_harder_braking(self, planner):
    """It is a floor, not a cap. It may only ever make braking gentler."""
    for d, v, ve in [(62.4, -4.36, 17.55), (100.0, -1.0, 20.0), (90.0, -2.0, 25.0)]:
      limit = planner.get_far_lead_brake_limit(lead(d, v), ve, ACCEL_MIN)
      if limit is not None:
        assert limit >= ACCEL_MIN


class TestKnownRampAnchorDefect:
  """PINNED, NOT ENDORSED. Read the ramp-anchor note in the FAR_LEAD_BRAKE_LIMIT_* block before
  touching this. The ramp interpolates from the caller's accel_min toward the limit, so when the
  caller passes something gentler than FAR_LEAD_BRAKE_LIMIT_ACCEL -- the call site passes
  output_accel_min, which is -0.5 on most frames of the fault -- intermediate ramp values land
  TIGHTER than the constant that is documented as the gentlest this may command.

  These tests exist so the inversion is visible rather than latent. Re-anchoring to ACCEL_MIN
  was measured and makes the feature nearly inert on its only positive example, because ttc is
  derived from the same vRel the fault corrupts. Do not change this without first replacing ttc
  with a confidence signal the fault does not corrupt."""

  def test_partial_ramp_is_tighter_than_the_documented_floor(self, planner):
    ve, d = 20.0, 80.0
    ttc = 12.0  # halfway between MIN_TTC and FULL_TTC, so ramp == 0.5
    limit = planner.get_far_lead_brake_limit(lead(d, -(d / ttc)), ve, -0.5)
    assert limit is not None
    assert limit > FAR_LEAD_BRAKE_LIMIT_ACCEL, (
      "known defect: with a gentle accel_min the partial ramp clamps harder than the full one"
    )

  def test_the_unit_test_anchor_and_the_call_site_disagree(self, planner):
    """Same geometry, two accel_min values, two different answers. This is why the original
    tests never caught the inversion: they only ever passed ACCEL_MIN."""
    ve, d, ttc = 20.0, 80.0, 12.0
    as_tested = planner.get_far_lead_brake_limit(lead(d, -(d / ttc)), ve, ACCEL_MIN)
    as_called = planner.get_far_lead_brake_limit(lead(d, -(d / ttc)), ve, -0.5)
    assert as_tested < FAR_LEAD_BRAKE_LIMIT_ACCEL < as_called


class TestRamp:
  """It must be a control law, not a step -- this file rejects discontinuities elsewhere."""

  def test_continuous_across_the_ttc_threshold(self, planner):
    ve, d = 20.0, 100.0
    just_below = planner.get_far_lead_brake_limit(
      lead(d, -(d / (FAR_LEAD_BRAKE_LIMIT_MIN_TTC - 0.05))), ve, ACCEL_MIN)
    just_above = planner.get_far_lead_brake_limit(
      lead(d, -(d / (FAR_LEAD_BRAKE_LIMIT_MIN_TTC + 0.05))), ve, ACCEL_MIN)
    assert just_below is None
    assert just_above == pytest.approx(ACCEL_MIN, abs=0.05), (
      "at the threshold the limit must equal accel_min, i.e. no restriction"
    )

  def test_monotonic_in_ttc(self, planner):
    ve, d = 20.0, 100.0
    prev = None
    for ttc in (10.5, 11.0, 12.0, 13.0, FAR_LEAD_BRAKE_LIMIT_FULL_TTC, 20.0):
      limit = planner.get_far_lead_brake_limit(lead(d, -(d / ttc)), ve, ACCEL_MIN)
      assert limit is not None
      if prev is not None:
        assert limit >= prev - 1e-9, "a longer TTC must never permit harder braking"
      prev = limit

  def test_saturates_at_the_full_limit(self, planner):
    ve, d = 20.0, 100.0
    for ttc in (FAR_LEAD_BRAKE_LIMIT_FULL_TTC, 30.0, 1e6):
      limit = planner.get_far_lead_brake_limit(lead(d, -(d / ttc)), ve, ACCEL_MIN)
      assert limit == pytest.approx(FAR_LEAD_BRAKE_LIMIT_ACCEL, abs=0.01)

  def test_opening_lead_is_fully_limited(self, planner):
    """No closing speed means no collision time -- the far end of the ramp, not an exemption."""
    limit = planner.get_far_lead_brake_limit(lead(80.0, +1.5), 20.0, ACCEL_MIN)
    assert limit == pytest.approx(FAR_LEAD_BRAKE_LIMIT_ACCEL, abs=0.01)


class TestDefaultOff:
  def test_param_default_is_off(self):
    """A TEST feature must not change behaviour for anyone who has not opted in."""
    from openpilot.common.params import Params
    p = Params()
    p.remove("FarLeadBrakeLimit")
    assert p.get_bool("FarLeadBrakeLimit") is False
