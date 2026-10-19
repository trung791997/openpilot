"""stock_long_planner: the camera-only Honda planner is upstream's min(lead MPC, cruise, e2e) with no extra guards."""
from types import SimpleNamespace

import pytest

from cereal import log
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.selfdrive.controls.lib import stock_long_planner as slp
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState


def lead(status=False, d_rel=0.0, v_lead=0.0):
  return SimpleNamespace(status=status, dRel=d_rel, yRel=0.0, vLead=v_lead, vLeadK=v_lead, aLeadK=0.0, aLeadTau=1.5,
                         modelProb=1.0 if status else 0.0, radar=False, vRel=0.0)


def make_sm(v_ego=20.0, v_cruise=25.0, e2e_a=0.0, exp=True, lead_one=None, standstill=False):
  return {
    'carControl': SimpleNamespace(orientationNED=[0.0, 0.0, 0.0]),
    'carState': SimpleNamespace(vEgo=v_ego, aEgo=0.0, vCruise=v_cruise * 3.6, standstill=standstill, steeringAngleDeg=0.0),
    'controlsState': SimpleNamespace(forceDecel=False, longControlState=LongCtrlState.pid),
    'selfdriveState': SimpleNamespace(enabled=True, experimentalMode=exp, personality=log.LongitudinalPersonality.standard),
    'liveParameters': SimpleNamespace(angleOffsetDeg=0.0),
    'modelV2': SimpleNamespace(meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[1.0, 1.0])),
                               action=SimpleNamespace(desiredAcceleration=e2e_a, shouldStop=False)),
    'radarState': SimpleNamespace(leadOne=lead_one or lead(), leadTwo=lead()),
    'starpilotPlan': SimpleNamespace(vCruise=v_cruise, tFollow=1.45),
  }


def camera_only_cp():
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_BOSCH)
  CP.radarUnavailable = True
  return CP


def test_selected_only_for_camera_only_honda():
  assert slp.uses_stock_long_planner(camera_only_cp())
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_BOSCH)
  CP.radarUnavailable = False
  assert not slp.uses_stock_long_planner(CP)
  assert not slp.uses_stock_long_planner(SimpleNamespace(brand="toyota", radarUnavailable=True))


def test_experimental_follows_the_model_brake():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=20.0)
  for _ in range(20):
    planner.update(make_sm(e2e_a=-0.8))
  assert planner.source == 'e2e'
  assert planner.output_a_target == pytest.approx(-0.8)


def test_chill_ignores_the_model():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=20.0)
  for _ in range(20):
    planner.update(make_sm(e2e_a=-0.8, exp=False))
  assert planner.source != 'e2e'
  assert planner.output_a_target > -0.1


def test_cruise_accel_is_jerk_limited():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=10.0)
  planner.update(make_sm(v_ego=10.0, v_cruise=30.0, e2e_a=2.0))
  assert planner.a_cruise <= slp.J_CRUISE_VALS[0] * planner.dt + 1e-6


def test_close_slow_lead_brakes_through_the_mpc():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=20.0)
  for _ in range(20):
    planner.update(make_sm(e2e_a=0.0, lead_one=lead(True, 25.0, 5.0)))
  assert planner.source == 'lead0'
  assert planner.output_a_target < -1.0


def test_weak_lead_cap_limits_the_creep_toward_a_slow_lead():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=2.0)
  for _ in range(60):
    planner.update(make_sm(v_ego=2.0, v_cruise=10.0, e2e_a=1.0, lead_one=lead(True, 12.0, 2.0)))
  cap = slp.get_weak_lead_accel_cap(make_sm(lead_one=lead(True, 12.0, 2.0))['radarState'], 2.0)
  assert cap is not None
  assert planner.output_a_target <= cap + 1e-6


def test_weak_lead_cap_never_softens_a_brake():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=2.0)
  for _ in range(20):
    planner.update(make_sm(v_ego=2.0, v_cruise=10.0, e2e_a=-1.5, lead_one=lead(True, 12.0, 2.0)))
  assert planner.output_a_target == pytest.approx(-1.5)


def test_weak_lead_cap_release_is_ramped():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=2.0)
  for _ in range(60):
    planner.update(make_sm(v_ego=2.0, v_cruise=10.0, e2e_a=1.0, lead_one=lead(True, 12.0, 2.0)))
  before = planner.output_a_target
  planner.update(make_sm(v_ego=2.0, v_cruise=10.0, e2e_a=1.0))
  assert planner.output_a_target <= max(before, 0.0) + slp.WEAK_LEAD_CAP_EASE_JERK * planner.dt + 1e-6


def test_brake_builds_no_faster_than_the_limit_without_a_closing_lead():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=20.0)
  planner.update(make_sm(e2e_a=0.0))
  prev = planner.output_a_target
  planner.update(make_sm(e2e_a=-2.0))
  assert planner.output_a_target == pytest.approx(prev - slp.BRAKE_BUILD_JERK * planner.dt)
  for _ in range(40):
    planner.update(make_sm(e2e_a=-2.0))
  assert planner.output_a_target == pytest.approx(-2.0)


def test_closing_lead_lifts_the_brake_limit():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=20.0)
  planner.update(make_sm(e2e_a=0.0))
  prev = planner.output_a_target
  planner.update(make_sm(e2e_a=-2.0, lead_one=lead(True, 25.0, 5.0)))
  assert planner.output_a_target < prev - 2 * slp.BRAKE_BUILD_JERK * planner.dt


def test_brake_release_is_not_slowed():
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=20.0)
  for _ in range(40):
    planner.update(make_sm(e2e_a=-2.0))
  planner.update(make_sm(e2e_a=-0.2))
  assert planner.output_a_target == pytest.approx(-0.2)


def test_gas_rises_no_faster_than_the_limit():
  assert slp.limit_output_jerk(1.0, 0.0, 0.05, False) == pytest.approx(slp.GAS_RISE_JERK * 0.05)
  assert slp.limit_output_jerk(1.0, -1.0, 0.05, False) == pytest.approx(slp.GAS_RISE_JERK * 0.05)


def test_model_lead_trajectory_reaches_the_mpc_when_on(monkeypatch):
  seen = {}
  planner = slp.StockLongitudinalPlanner(camera_only_cp(), init_v=20.0)
  original = planner.mpc.update

  def spy(*args, **kwargs):
    seen['modelV2'] = kwargs.get('modelV2')
    return original(*args, **{k: v for k, v in kwargs.items() if k != 'modelV2'})

  monkeypatch.setattr(planner.mpc, 'update', spy)
  sm = make_sm()
  planner.update(sm)
  assert seen['modelV2'] is None
  monkeypatch.setattr(slp, 'USE_MODEL_LEAD_TRAJECTORY', True)
  planner.update(sm)
  assert seen['modelV2'] is sm['modelV2']
