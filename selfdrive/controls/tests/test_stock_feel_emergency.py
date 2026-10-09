"""STOCK_FEEL_EMERGENCY (longitudinal_planner): Stock Brake Feel steps aside for a hard-braking lead. 000002f5. Static only."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


def lead(d, v_rel, v_lead, a_lead):
  return SimpleNamespace(status=True, dRel=d, vRel=v_rel, vLead=v_lead, aLeadK=a_lead)


# 000002f5 radarState leadOne (track 5) and vEgo, 10:01 into the route.
F5 = [  # t, vEgo, dRel, vRel, vLead, aLeadK
  (601.05, 18.82, 38.6, -1.5, 17.4, -0.98),
  (601.25, 18.72, 37.5, -2.1, 16.8, -1.35),
  (601.45, 18.58, 35.6, -4.1, 14.6, -2.46),
  (601.66, 18.44, 33.8, -6.1, 12.4, -4.28),
  (601.86, 18.23, 32.1, -7.8, 10.6, -5.80),
]


def test_on_by_default():
  assert lp.STOCK_FEEL_EMERGENCY


def test_2f5_latches_on_the_hard_braking_lead():
  active = False
  first = None
  for t, v, d, vr, vl, al in F5:
    active = lp.stock_feel_emergency([lead(d, vr, vl, al)], v, active)
    if active and first is None:
      first = t
  assert first == 601.45  # 1.4 s before the driver braked at 602.88


def test_2f5_target_passes_through_once_latched(monkeypatch):
  monkeypatch.setattr(lp, 'STOCK_FEEL_LEAD_STOP', False)  # the plain stock law; stop tests A and C turn this on
  t, v, d, vr, vl, al = F5[3]
  leads = [lead(d, vr, vl, al)]
  capped = lp.stock_feel_target(leads, -0.57, -4.5, 0.05, v)
  assert capped > -0.65  # the stock law: about 1 m/s^3 at TTC 5.5
  assert lp.stock_feel_target(leads, -0.57, -4.5, 0.05, v, emergency=True) == -4.5


def test_latch_holds_while_closing_and_clears_when_not():
  braking = lead(30.0, -6.0, 12.0, -4.0)
  assert lp.stock_feel_emergency([braking], 18.0, False)
  eased = lead(25.0, -3.0, 15.0, 0.5)  # lead stopped braking but the gap still closes
  assert lp.stock_feel_emergency([eased], 18.0, True)
  assert not lp.stock_feel_emergency([eased], 18.0, False)
  opening = lead(25.0, 0.3, 18.3, 0.5)
  assert not lp.stock_feel_emergency([opening], 18.0, True)


@pytest.mark.parametrize("l", [
  lead(32.3, -0.6, 16.7, 0.1),   # 2f5 570 s: steady follow
  lead(29.6, -0.8, 15.3, -0.97),  # 2f5 574.9 s: lead brushing the brake
  lead(60.0, -1.0, 19.0, -1.2),   # mild braking far lead: need under STOCK_FEEL_EMERGENCY_NEED
])
def test_normal_following_stays_on_the_stock_law(l):
  assert not lp.stock_feel_emergency([l], 17.0, False)


def test_hard_braking_lead_that_is_not_closing_is_ignored():
  assert not lp.stock_feel_emergency([lead(30.0, 0.2, 18.2, -3.0)], 18.0, False)


def test_switch_off(monkeypatch):
  monkeypatch.setattr(lp, 'STOCK_FEEL_EMERGENCY', False)
  assert not lp.stock_feel_emergency([lead(*F5[3][2:])], F5[3][1], True)
