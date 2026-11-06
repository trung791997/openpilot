"""Static test for Stock Brake Feel's far depth entry (owner 2026-11-06, route 96 32:17). Not road evidence."""
from types import SimpleNamespace

from openpilot.starpilot.longitudinal import honda_bosch_a_long as hbl

DT = 0.05


def lead(d, v_rel, a_lead=0.0):
  return SimpleNamespace(present=True, dRel=d, vRel=v_rel, aLeadK=a_lead)


def test_far_lead_cap_is_minus_0_6():
  # TTC 30 s: the model's -1.0 is held to -0.6 (was -0.35), reached at the jerk-table rate
  leads = (lead(150.0, -5.0),)
  prev = -0.6
  assert hbl.stock_feel_target(leads, prev, -1.0, DT, v_ego=20.0) == -0.6


def test_far_cap_never_adds_braking():
  leads = (lead(150.0, -5.0),)
  assert hbl.stock_feel_target(leads, -0.2, -0.2, DT, v_ego=20.0) == -0.2
