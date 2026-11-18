"""Newborn lead aLeadK bound (D-080, NEWBORN_HOLD_ENABLED): pure python. The class and bound_newborn_leads were restored
verbatim from 44f382face; the original had no unit tests (only an ad hoc fuzz), so these are new. Replay only."""
from types import SimpleNamespace

import pytest

from openpilot.starpilot.longitudinal import honda_bosch_a_long as hb


def lead(d, a, tid=7, radar=True, v=20.0, present=True):
  return SimpleNamespace(present=present, radar=radar, radarTrackId=tid, dRel=d, vLead=v, vRel=v - 20.0, aLeadK=a)


def rs(l1, l2=None):
  return SimpleNamespace(leadOne=l1, leadTwo=l2 if l2 is not None else lead(0.0, 0.0, present=False), marker='rs')


def test_newborn_aleadk_held_at_min_brake():
  h = hb.NewbornLeadHold()
  out = hb.bound_newborn_leads(rs(lead(61.0, -2.39)), 0.0, h)
  assert out.leadOne.aLeadK == -hb.NEWBORN_LEAD_MIN_BRAKE
  assert out.leadOne.dRel == 61.0 and out.leadOne.radarTrackId == 7  # only aLeadK changes
  assert out.marker == 'rs' and h.bound_frames == 1


def test_gentle_lead_untouched_returns_same_object():
  h = hb.NewbornLeadHold()
  r = rs(lead(61.0, -0.4))
  assert hb.bound_newborn_leads(r, 0.0, h) is r


def test_bound_ends_after_newborn_frames():
  h = hb.NewbornLeadHold()
  for _ in range(hb.NEWBORN_LEAD_FRAMES):
    out = hb.bound_newborn_leads(rs(lead(61.0, -2.39)), 0.0, h)
    assert out.leadOne.aLeadK == -1.0
  r = rs(lead(61.0, -2.39))
  assert hb.bound_newborn_leads(r, 0.0, h) is r  # no longer newborn


def test_vision_and_absent_leads_untouched():
  h = hb.NewbornLeadHold()
  r = rs(lead(61.0, -2.39, radar=False))
  assert hb.bound_newborn_leads(r, 0.0, h) is r
  r = rs(lead(61.0, -2.39, present=False))
  assert hb.bound_newborn_leads(r, 0.0, h) is r


def test_range_fit_confirms_hard_braking():
  # range history that really is decelerating at -3 m/s^2 relative (d = 61 - 0.5*3*t^2 shape), ego a = 0
  h = hb.NewbornLeadHold()
  out = None
  for i in range(hb.NEWBORN_LEAD_FIT_FRAMES + 2):
    t = i * hb.NEWBORN_LEAD_DT
    out = hb.bound_newborn_leads(rs(lead(61.0 - 1.5 * t * t, -3.0)), 0.0, h)
  assert out.leadOne.aLeadK == pytest.approx(-3.0, abs=0.05)  # fit lowers the floor to its own estimate: -3 passes (<= -1)
  # and a steady range with a claimed -3 stays at -1.0
  h = hb.NewbornLeadHold()
  for _ in range(hb.NEWBORN_LEAD_FIT_FRAMES + 2):
    out = hb.bound_newborn_leads(rs(lead(61.0, -3.0)), 0.0, h)
  assert out.leadOne.aLeadK == -1.0


def test_gap_and_range_jump_make_track_newborn_again():
  h = hb.NewbornLeadHold()
  for _ in range(hb.NEWBORN_LEAD_FRAMES + 5):
    hb.bound_newborn_leads(rs(lead(61.0, -2.0)), 0.0, h)
  for _ in range(hb.NEWBORN_LEAD_GAP_FRAMES + 2):  # absent
    hb.bound_newborn_leads(rs(lead(0.0, 0.0, present=False)), 0.0, h)
  assert hb.bound_newborn_leads(rs(lead(61.0, -2.0)), 0.0, h).leadOne.aLeadK == -1.0
  h = hb.NewbornLeadHold()
  for _ in range(hb.NEWBORN_LEAD_FRAMES + 5):
    hb.bound_newborn_leads(rs(lead(61.0, -2.0)), 0.0, h)
  assert hb.bound_newborn_leads(rs(lead(61.0 + hb.NEWBORN_LEAD_JUMP_M + 0.5, -2.0)), 0.0, h).leadOne.aLeadK == -1.0


def test_wiring_follows_switch_at_construction(monkeypatch):
  monkeypatch.setattr(hb, 'active_for', lambda CP: True)
  cp = SimpleNamespace(radarUnavailable=False)
  monkeypatch.setattr(hb, 'NEWBORN_HOLD_ENABLED', False)
  off = hb.HondaBoschALongEnhancement(cp, 0.05)
  assert off.newborn_hold is None
  r = rs(lead(61.0, -2.39))
  assert off.planner_radar_state(r, 0.0) is r
  monkeypatch.setattr(hb, 'NEWBORN_HOLD_ENABLED', True)
  on = hb.HondaBoschALongEnhancement(cp, 0.05)
  assert on.newborn_hold is not None
  assert on.planner_radar_state(r, 0.0).leadOne.aLeadK == -1.0
  monkeypatch.setattr(hb, 'active_for', lambda CP: False)
  assert hb.HondaBoschALongEnhancement(cp, 0.05).newborn_hold is None


def mv(x, v=12.0, a=-0.25, prob=0.98):
  return SimpleNamespace(leadsV3=[SimpleNamespace(prob=prob, x=[x], v=[v], a=[a])])


def test_camera_corroboration_prevents_converging_range_fit_from_tripping_emergency():
  # ed seg 6 shape: converging newborn track (65 -> 45 m, TTC > 4.2 s) with calm matching camera (1-frame -0.66 spike)
  h = hb.NewbornLeadHold()
  d = 65.0
  bounded = None
  for i in range(hb.NEWBORN_LEAD_FRAMES + 4):
    # Quadratic convergence d = 65 - 1.5*t^2 produces fake negative _range_decel
    t = i * hb.NEWBORN_LEAD_DT
    d = 65.0 - 1.5 * t * t
    l = SimpleNamespace(present=True, radar=True, radarTrackId=6, dRel=d, vLead=8.0, vRel=-10.0, aLeadK=-2.75)
    a_cam = -0.66 if i == 37 else -0.25
    bounded = hb.bound_newborn_leads(rs(l, l), -0.4, h, model_v2=mv(d + 5.0, a=a_cam))
    if i < hb.NEWBORN_LEAD_FRAMES:
      assert bounded.leadOne.aLeadK == pytest.approx(-hb.NEWBORN_MODEL_MIN_BRAKE)
      assert bounded.leadTwo.aLeadK == pytest.approx(-hb.NEWBORN_MODEL_MIN_BRAKE)
    else:
      # Smooth D-042 slew-out after NEWBORN_LEAD_FRAMES, never stepping straight to -2.75 (and no double-step from leadTwo)
      expected = -hb.NEWBORN_MODEL_MIN_BRAKE - (i - hb.NEWBORN_LEAD_FRAMES + 1) * hb.NEWBORN_RELEASE_RATE * hb.NEWBORN_LEAD_DT
      assert bounded.leadOne.aLeadK == pytest.approx(expected)
      assert bounded.leadTwo.aLeadK == pytest.approx(expected)


def test_camera_hard_brake_and_close_urgency_keep_full_authority():
  # (a) When matching camera ALSO sees hard braking (-3.0), range fit lowers the floor to -3.0
  h = hb.NewbornLeadHold()
  out = None
  for i in range(hb.NEWBORN_LEAD_FIT_FRAMES + 4):
    t = i * hb.NEWBORN_LEAD_DT
    d = 61.0 - 1.5 * t * t
    l = SimpleNamespace(present=True, radar=True, radarTrackId=7, dRel=d, vLead=12.0, vRel=-8.0, aLeadK=-3.0)
    out = hb.bound_newborn_leads(rs(l), 0.0, h, model_v2=mv(d + 1.5, a=-3.0))
  assert out.leadOne.aLeadK == pytest.approx(-3.0, abs=0.05)

  # (b) When range drops < NEWBORN_MODEL_MIN_D (40 m) or TTC <= 4 s, urgent leads bypass camera floor
  h2 = hb.NewbornLeadHold()
  for i in range(hb.NEWBORN_LEAD_FIT_FRAMES + 4):
    t = i * hb.NEWBORN_LEAD_DT
    d = 35.0 - 1.5 * t * t
    l = SimpleNamespace(present=True, radar=True, radarTrackId=8, dRel=d, vLead=12.0, vRel=-8.0, aLeadK=-3.0)
    out = hb.bound_newborn_leads(rs(l), 0.0, h2, model_v2=mv(d + 1.5, a=-0.1))
  assert out.leadOne.aLeadK == pytest.approx(-3.0, abs=0.05)
