"""Stock Brake Feel lead brake anticipation: a clearly braking radar lead floors the target earlier; only adds braking."""
from types import SimpleNamespace

from openpilot.starpilot.longitudinal import honda_bosch_a_long as hbl
from openpilot.starpilot.longitudinal.honda_bosch_a_long import (ANT_MAX, ANT_ONSET_JERK, ANT_RELEASE_JERK,
                                                                 lead_brake_anticipation_need,
                                                                 lead_brake_anticipation_target)

DT = 0.05
NONE = SimpleNamespace(present=False, radar=False, dRel=0.0, vRel=0.0, vLead=0.0, aLeadK=0.0)


def lead(d=47.0, v_lead=14.2, a_lead=-2.2, v_ego=19.0, radar=True):
  return SimpleNamespace(present=True, radar=radar, dRel=d, vRel=v_lead - v_ego, vLead=v_lead, aLeadK=a_lead)


def test_route82_braking_lead_needs_firm_brake():
  # route 82 rt ~360.4: lead at 47 m, 14.2 m/s and braking, we are still at 19 m/s
  need = lead_brake_anticipation_need(lead(), 19.0, -0.5)
  assert 1.0 < need < 2.5
  harder = lead_brake_anticipation_need(lead(v_lead=13.8, a_lead=-3.0, d=46.0), 18.8, -1.0)
  assert harder > need


def test_not_armed():
  assert lead_brake_anticipation_need(lead(a_lead=-1.2), 19.0, 0.0) == 0.0         # gentle slowdown
  assert lead_brake_anticipation_need(lead(a_lead=-2.2), 19.0, -1.5) == 0.0        # rail guard: we brake as hard
  assert lead_brake_anticipation_need(lead(radar=False), 19.0, 0.0) == 0.0         # vision lead
  assert lead_brake_anticipation_need(lead(v_lead=30.0, a_lead=-2.0), 19.0, 0.0) == 0.0  # ends faster than us
  assert lead_brake_anticipation_need(NONE, 19.0, 0.0) == 0.0


def test_need_capped():
  assert lead_brake_anticipation_need(lead(d=5.0, v_lead=10.0, a_lead=-3.0), 25.0, 0.0) == ANT_MAX


def test_onset_and_release_are_jerk_limited_and_never_raise_target():
  prev, floor = 0.0, 0.0
  leads = (lead(), NONE)
  for _ in range(5):
    out, floor = lead_brake_anticipation_target(leads, prev, 0.3, DT, 19.0, -0.5, floor)
    assert out >= prev - ANT_ONSET_JERK * DT - 1e-9
    assert out <= 0.3
    prev = out
  assert prev < 0.0
  # lead stops braking: the floor lets off at the release rate, never jumps
  calm = (lead(a_lead=0.0), NONE)
  out, new_floor = lead_brake_anticipation_target(calm, prev, 0.3, DT, 19.0, -0.5, floor)
  assert abs(new_floor - (floor + ANT_RELEASE_JERK * DT)) < 1e-9
  # a deeper planner target always passes through
  out, _ = lead_brake_anticipation_target(leads, prev, -3.0, DT, 19.0, -0.5, floor)
  assert out == -3.0


def test_only_with_stock_brake_feel(monkeypatch):
  monkeypatch.setattr(hbl, 'active_for', lambda CP: True)
  kwargs = {'lead_one': lead(), 'lead_two': NONE, 'prev': 0.0, 'target': 0.3, 'v_ego': 19.0, 'stopping': False,
            'standstill': False, 'reset_state': False, 'a_ego': -0.5}
  off = hbl.HondaBoschALongEnhancement(None, DT, stock_brake_feel=False)
  assert off.step(**kwargs) == 0.3
  on = hbl.HondaBoschALongEnhancement(None, DT, stock_brake_feel=True)
  assert on.step(**kwargs) < 0.0
  assert on.anticipation_floor < 0.0
  on.step(**{**kwargs, 'reset_state': True})
  assert on.anticipation_floor == 0.0
