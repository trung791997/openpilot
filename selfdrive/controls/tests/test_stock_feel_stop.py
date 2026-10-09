"""STOCK_FEEL_LEAD_STOP and STOP_EASE (longitudinal_planner): 000002f2 stop findings. Static only."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


def lead(d, v_rel, v_lead, a_lead):
  return SimpleNamespace(status=True, dRel=d, vRel=v_rel, vLead=v_lead, aLeadK=a_lead)


@pytest.fixture
def lead_stop_on(monkeypatch):
  monkeypatch.setattr(lp, 'STOCK_FEEL_LEAD_STOP', True)


@pytest.fixture
def ease_on(monkeypatch):
  monkeypatch.setattr(lp, 'STOP_EASE', True)


def test_variant_switches():
  assert lp.STOCK_FEEL_LEAD_STOP is True and lp.STOP_EASE is True


def test_braking_lead_keeps_planner_depth(lead_stop_on):
  # 2f2 1099 at -7.3 s: 18.25 m/s, lead 29.1 m closing 3.65 at 14.8 m/s and braking ~-3.5; TTC 8 s caps at -1.6.
  braking = lead(29.1, -3.65, 14.8, -3.5)
  assert lp.lead_stop_need([braking], 18.25) > 1.6
  need = lp.lead_stop_need([braking], 18.25)
  assert lp.stock_feel_target([braking], -1.0, -3.0, 0.05, 18.25) == pytest.approx(-1.25)  # 5 m/s^3 from -1.0
  assert lp.stock_feel_target([braking], -2.9, -3.5, 0.05, 18.25) == pytest.approx(-need)  # capped at the need
  assert lp.stock_feel_target([braking], -1.0, -0.8, 0.05, 18.25) == -0.8  # a shallower planner target stands


def test_steady_lead_keeps_the_stock_cap(lead_stop_on):
  steady = lead(29.1, -3.65, 14.8, 0.0)
  assert lp.lead_stop_need([steady], 18.25) == 0.0
  assert lp.stock_feel_target([steady], -1.6, -3.0, 0.05, 18.25) > -1.7


def test_mild_braking_far_lead_is_capped_at_its_need(lead_stop_on):
  far = lead(60.0, -1.0, 19.0, -1.2)
  need = lp.lead_stop_need([far], 20.0)
  assert 0.35 < need < 1.2  # past the TTC-60 table depth (-0.35), so the cap is the need, not the planner's -3
  assert lp.stock_feel_target([far], -need, -3.0, 0.05, 20.0) == pytest.approx(-need)


def test_ease_floor_shape(ease_on):
  assert lp.stop_ease_floor(3.5, None) is None
  assert lp.stop_ease_floor(1.0, None) == pytest.approx(-1.1)
  assert lp.stop_ease_floor(0.0, None) == pytest.approx(-0.5)


def test_ease_needs_room_to_the_lead(ease_on):
  # 2f2 1099 at 2.57 m/s with the lead 3.5 m ahead: easing would end inside 2 m, so the brake is left alone.
  assert lp.stop_ease_floor(2.57, 3.5) is None
  assert lp.stop_ease_floor(1.0, 6.0) == pytest.approx(-1.1)


def test_eased_target_rises_at_the_jerk_limit(ease_on):
  floor = lp.stop_ease_floor(0.8, None)
  assert lp.stop_eased_target(-2.27, -2.27, floor, 0.05) == pytest.approx(-2.27 + lp.STOP_EASE_JERK * 0.05)
  assert lp.stop_eased_target(-1.0, -2.27, floor, 0.05) == pytest.approx(floor)
  assert lp.stop_eased_target(-0.5, -0.4, floor, 0.05) == -0.4  # above the floor: untouched
  assert lp.stop_eased_target(-2.27, -2.27, None, 0.05) == -2.27
