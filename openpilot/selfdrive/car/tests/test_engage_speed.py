import math
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import E2E_ENGAGE_SPEED_KPH, VCruiseHelper, engage_initial_speed, engage_keeps_speed
from opendbc.car.structs import car


def test_only_stock_planner_experimental_gets_66_mph():
  assert engage_initial_speed(40, True, True) == E2E_ENGAGE_SPEED_KPH
  assert engage_initial_speed(40, False, True) == 40  # Chill keeps the saved speed
  assert engage_initial_speed(40, True, False) == 40  # StarPilot planner keeps the saved speed
  assert round(E2E_ENGAGE_SPEED_KPH * CV.KPH_TO_MPH) == 66


def _engage(v_ego_mph, initial_speed, keep_speed=False):
  helper = VCruiseHelper(car.CarParams(pcmCruise=False))
  cs = SimpleNamespace(vEgo=v_ego_mph * CV.MPH_TO_MS, buttonEvents=[])
  helper.initialize_v_cruise(cs, True, resume=False, initial_speed=initial_speed, keep_speed=keep_speed)
  return helper.v_cruise_kph * CV.KPH_TO_MPH


def test_engage_sets_66_or_current_speed_if_faster():
  assert round(_engage(30, E2E_ENGAGE_SPEED_KPH)) == 66
  assert round(_engage(75, E2E_ENGAGE_SPEED_KPH)) == 75
  assert round(_engage(30, 40)) == 30  # Chill: engage speed


def test_only_stock_planner_chill_keeps_speed():
  assert engage_keeps_speed(False, True)
  assert not engage_keeps_speed(True, True)  # Experimental: 66 mph rule
  assert not engage_keeps_speed(False, False)  # StarPilot planner unchanged


def test_chill_engage_below_25_mph_keeps_current_speed():
  assert round(_engage(13, 40, keep_speed=True)) == 13
  assert round(_engage(30, 40, keep_speed=True)) == 30
  assert round(_engage(13, 40)) == 25  # without it, the 40 kph floor applies
  assert round(_engage(2, 40, keep_speed=True)) == 25  # near standstill falls back to the saved speed


def _engage_at_limit(v_ego_mph, target_mph, resume=False, keep_speed=False, metric=False):
  helper = VCruiseHelper(car.CarParams(pcmCruise=False))
  helper.v_cruise_kph = helper.v_cruise_kph_last = 50 * CV.MPH_TO_KPH  # a set speed from an earlier engagement
  cs = SimpleNamespace(vEgo=v_ego_mph * CV.MPH_TO_MS, buttonEvents=[])
  helper.initialize_v_cruise(cs, False, resume=resume, initial_speed=40, keep_speed=keep_speed,
                             slc_target_mps=None if target_mph is None else target_mph * CV.MPH_TO_MS, is_metric=metric)
  return helper.v_cruise_kph * CV.KPH_TO_MPH, helper.v_cruise_cluster_kph * CV.KPH_TO_MPH


def test_set_engages_at_the_speed_limit_target():
  # SET with a limit (plus offset) available engages at it, slower or faster than the car.
  assert [round(v) for v in _engage_at_limit(30, 45)] == [45, 45]
  assert [round(v) for v in _engage_at_limit(70, 55)] == [55, 55]
  assert round(_engage_at_limit(13, 35, keep_speed=True)[0]) == 35
  assert round(_engage_at_limit(30, 45, metric=True)[0] * CV.MPH_TO_KPH) == round(45 * CV.MPH_TO_KPH)


def test_resume_and_missing_limit_keep_existing_engage_speed():
  assert round(_engage_at_limit(30, 45, resume=True)[0]) == 50  # RES restores the last set speed
  assert round(_engage_at_limit(30, None)[0]) == 30
  for bad in (float('nan'), float('inf'), 0., -5.):
    helper = VCruiseHelper(car.CarParams(pcmCruise=False))
    cs = SimpleNamespace(vEgo=30 * CV.MPH_TO_MS, buttonEvents=[])
    helper.initialize_v_cruise(cs, False, resume=False, initial_speed=40, slc_target_mps=bad)
    assert round(helper.v_cruise_kph * CV.KPH_TO_MPH) == 30


def test_pcm_cruise_ignores_the_limit_target():
  helper = VCruiseHelper(car.CarParams(pcmCruise=True))
  before = helper.v_cruise_kph
  helper.initialize_v_cruise(SimpleNamespace(vEgo=10., buttonEvents=[]), False, resume=False, slc_target_mps=20.)
  assert helper.v_cruise_kph == before or (math.isnan(before) and math.isnan(helper.v_cruise_kph))
