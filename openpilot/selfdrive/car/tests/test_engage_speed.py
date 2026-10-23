from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import E2E_ENGAGE_SPEED_KPH, VCruiseHelper, engage_initial_speed
from opendbc.car.structs import car


def test_only_stock_planner_experimental_gets_66_mph():
  assert engage_initial_speed(40, True, True) == E2E_ENGAGE_SPEED_KPH
  assert engage_initial_speed(40, False, True) == 40  # Chill keeps the saved speed
  assert engage_initial_speed(40, True, False) == 40  # StarPilot planner keeps the saved speed
  assert round(E2E_ENGAGE_SPEED_KPH * CV.KPH_TO_MPH) == 66


def _engage(v_ego_mph, initial_speed):
  helper = VCruiseHelper(car.CarParams(pcmCruise=False))
  cs = SimpleNamespace(vEgo=v_ego_mph * CV.MPH_TO_MS, buttonEvents=[])
  helper.initialize_v_cruise(cs, True, resume=False, initial_speed=initial_speed)
  return helper.v_cruise_kph * CV.KPH_TO_MPH


def test_engage_sets_66_or_current_speed_if_faster():
  assert round(_engage(30, E2E_ENGAGE_SPEED_KPH)) == 66
  assert round(_engage(75, E2E_ENGAGE_SPEED_KPH)) == 75
  assert round(_engage(30, 40)) == 30  # Chill: engage speed
