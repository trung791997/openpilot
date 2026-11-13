from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.accel_boost import (
  ACCEL_BOOST_INITIAL, ACCEL_BOOST_MAX, ACCEL_BOOST_MIN_SPEED, ACCEL_BOOST_PER_OVERRIDE, LEAD_GATE_FADE_IN_S, LEAD_GATE_FADE_OUT_S,
  AccelBoost,
)

V = 17.0


Personality = log.LongitudinalPersonality


def make_radar_state(lead=None):
  # Real cereal message, so a wrong field name fails here instead of silently reading as no lead.
  rs = log.RadarState.new_message()
  if lead is not None:
    present, d_rel, v_rel, a_lead = lead
    rs.leadOne.present = present
    rs.leadOne.dRel, rs.leadOne.vRel, rs.leadOne.aLeadK = d_rel, v_rel, a_lead
  return rs


def make_lead(status=False, d_rel=0.0, v_rel=0.0, a_lead=0.0):
  return (status, d_rel, v_rel, a_lead)


def make_sm(enabled=True, gas=False, v_ego=V, exp=True, personality=Personality.standard, lead=None):
  return {'selfdriveState': SimpleNamespace(enabled=enabled, experimentalMode=exp, personality=personality),
          'carState': SimpleNamespace(gasPressed=gas, vEgo=v_ego),
          'radarState': make_radar_state(lead)}


def run(b, seconds, e2e=0.0, mpc=1.0, a_cruise=1.0, **kw):
  out = None
  for _ in range(round(seconds / DT_MDL)):
    out = b.update(make_sm(**kw), e2e, mpc, a_cruise)
  return out


def test_one_press_adds_per_override_and_stops():
  b = AccelBoost()
  run(b, 1.0, gas=False)
  run(b, 5.0, gas=True)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_PER_OVERRIDE)


def test_presses_accumulate_to_max_and_value_holds_after_release():
  b = AccelBoost()
  for _ in range(6):
    run(b, 0.5, gas=False)
    run(b, 5.0, gas=True)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_MAX)
  run(b, 2.0, gas=False)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_MAX)


def test_eligibility_is_latched_before_the_press():
  b = AccelBoost()
  run(b, 1.0, gas=False, mpc=0.0, a_cruise=0.0)  # not model limited
  run(b, 2.0, gas=True)  # limited during the press, but not at press start
  assert b.total_boost == 0.0
  run(b, 1.0, gas=False)
  run(b, 5.0, gas=True, mpc=0.0)  # press-time lead plan changes cannot revoke it
  assert b.total_boost == pytest.approx(ACCEL_BOOST_PER_OVERRIDE)


def test_needs_margin():
  b = AccelBoost()
  run(b, 1.0, e2e=0.95, mpc=1.0)  # within the 0.1 margin
  run(b, 2.0, e2e=0.95, mpc=1.0, gas=True)
  assert b.total_boost == 0.0


def test_press_counts_outside_experimental_mode():
  # commaai/openpilot#39097 dropped the Experimental Mode check; the planners only use the e2e target in Experimental Mode
  b = AccelBoost()
  run(b, 1.0, exp=False)
  run(b, 2.0, exp=False, gas=True)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_PER_OVERRIDE)


def test_aggressive_starts_at_initial_boost_and_keeps_it():
  # commaai/openpilot#39229
  b = AccelBoost()
  run(b, 0.1, personality=Personality.aggressive)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_INITIAL)
  run(b, 0.1, enabled=False, personality=Personality.aggressive)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_INITIAL)
  run(b, 1.0, personality=Personality.aggressive)
  run(b, 5.0, gas=True, personality=Personality.aggressive)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_INITIAL + ACCEL_BOOST_PER_OVERRIDE)
  run(b, 0.1, personality=Personality.standard)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_INITIAL + ACCEL_BOOST_PER_OVERRIDE)
  run(b, 0.1, enabled=False, personality=Personality.standard)
  assert b.total_boost == 0.0


def test_speed_ramp_scales_buildup_and_output():
  b = AccelBoost()
  run(b, 1.0, v_ego=ACCEL_BOOST_MIN_SPEED)
  run(b, 2.0, v_ego=ACCEL_BOOST_MIN_SPEED, gas=True)
  assert b.total_boost == 0.0  # no buildup at the minimum speed
  b.total_boost = 0.2
  assert b.update(make_sm(v_ego=ACCEL_BOOST_MIN_SPEED), 0.0, 1.0, 1.0) == 0.0
  assert b.update(make_sm(v_ego=1.5 * ACCEL_BOOST_MIN_SPEED), 0.0, 1.0, 1.0) == pytest.approx(0.1)
  assert b.update(make_sm(v_ego=2 * ACCEL_BOOST_MIN_SPEED), 0.0, 1.0, 1.0) == pytest.approx(0.2)


def test_clears_when_disabled():
  b = AccelBoost()
  run(b, 1.0)
  run(b, 5.0, gas=True)
  assert b.total_boost > 0.0
  run(b, 0.1, enabled=False)
  assert b.total_boost == 0.0 and not b.boost_eligible


def test_output_shape():
  b = AccelBoost()
  b.total_boost = 0.2
  def f(e2e):
    return b.update(make_sm(), e2e, 9.0, 9.0)

  assert f(-1.5) == -1.5  # at or below -1.0: no boost
  assert f(-1.0) == pytest.approx(-1.0)
  assert f(-0.5) == pytest.approx(-0.3)
  assert f(0.5) == pytest.approx(0.7)
  assert f(-0.75) == pytest.approx(-0.75 + 0.1)
  assert f(6.0) == 6.0


def test_lead_gate_fades_boost_toward_a_slowing_lead_and_back():
  b = AccelBoost()
  agg = Personality.aggressive
  assert run(b, 1.0, personality=agg) == pytest.approx(ACCEL_BOOST_INITIAL)
  closing = make_lead(True, d_rel=3.0 * V, v_rel=-1.0)
  assert run(b, LEAD_GATE_FADE_OUT_S, personality=agg, lead=closing) == pytest.approx(0.0, abs=1e-6)
  assert b.total_boost == pytest.approx(ACCEL_BOOST_INITIAL)  # kept, only the applied boost is gated
  assert run(b, LEAD_GATE_FADE_IN_S, personality=agg) == pytest.approx(ACCEL_BOOST_INITIAL)


@pytest.mark.parametrize('lead, gated', [
  (make_lead(True, d_rel=3.0 * V, v_rel=0.0, a_lead=-1.0), True),   # braking lead
  (make_lead(True, d_rel=5.0 * V, v_rel=-3.0), False),               # beyond the headway gate
  (make_lead(True, d_rel=2.0 * V, v_rel=-0.3, a_lead=-0.3), False),  # steady lead
  (make_lead(False, d_rel=1.0 * V, v_rel=-3.0), False),              # no lead
])
def test_lead_gate_conditions(lead, gated):
  b = AccelBoost()
  out = run(b, 1.0, personality=Personality.aggressive, lead=lead)
  assert out == pytest.approx(0.0 if gated else ACCEL_BOOST_INITIAL, abs=1e-6)
