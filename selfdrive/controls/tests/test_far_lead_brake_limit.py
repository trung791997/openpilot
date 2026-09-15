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
