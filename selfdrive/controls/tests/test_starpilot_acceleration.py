import math
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN
from openpilot.starpilot.common.accel_profile import A_CRUISE_MAX_BP_CUSTOM, ACCELERATION_PROFILES, DECELERATION_PROFILES
from openpilot.starpilot.controls.lib.starpilot_acceleration import (
  A_CRUISE_MIN_ECO,
  A_CRUISE_MIN_SPORT,
  A_CRUISE_MIN_TRAFFIC,
  PULSE_GLIDE_COAST_MIN_ACCEL,
  StarPilotAcceleration,
  get_max_accel_eco,
  get_max_accel_low_speeds,
  get_max_accel_ramp_off,
  get_max_accel_standard,
  get_max_accel_traffic,
  get_slc_shaped_min_accel,
)


class FakePlanner:
  def __init__(self, *, v_cruise=0.0, slc_target=0.0, slc_offset=0.0, overridden_speed=0.0,
               red_light=False, forcing_stop=False, disable_throttle=False):
    self.v_cruise = v_cruise
    self.starpilot_weather = SimpleNamespace(weather_id=0, reduce_acceleration=0.0)
    self.starpilot_vcruise = SimpleNamespace(
      slc_target=slc_target,
      slc_offset=slc_offset,
      forcing_stop=forcing_stop,
      slc=SimpleNamespace(overridden_speed=overridden_speed),
    )
    self.starpilot_cem = SimpleNamespace(stop_light_detected=red_light)
    self.starpilot_following = SimpleNamespace(disable_throttle=disable_throttle)


def make_toggles(**overrides):
  defaults = {
    "acceleration_profile": ACCELERATION_PROFILES["STANDARD"],
    "deceleration_profile": DECELERATION_PROFILES["ECO"],
    "custom_accel_profile": False,
    "custom_accel_profile_breakpoints": A_CRUISE_MAX_BP_CUSTOM,
    "custom_accel_profile_values": [],
    "ev_tuning": True,
    "truck_tuning": False,
    "map_acceleration": False,
    "map_deceleration": False,
    "set_speed_limit": True,
    "set_speed_offset": 0,
    "speed_limit_controller": True,
    "pulse_glide_speed_delta": 0.0,
  }
  defaults.update(overrides)
  return SimpleNamespace(**defaults)


def make_lead(status=False, d_rel=150.0, v_lead=0.0, a_lead_k=0.0):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=v_lead, aLeadK=a_lead_k)


def make_sm(*, set_speed_kph=100.0, lead_one=None, lead_two=None, standstill=False, force_decel=False,
            eco_gear=False, sport_gear=False, force_coast=False, pulse_and_glide=False, traffic_mode=False,
            v_ego_cluster=0.0, pitch=0.0, gas_pressed=False):
  return {
    "carState": SimpleNamespace(vCruise=set_speed_kph, standstill=standstill, vEgoCluster=v_ego_cluster,
                                gasPressed=gas_pressed),
    "carControl": SimpleNamespace(orientationNED=[0.0, pitch, 0.0]),
    "controlsState": SimpleNamespace(forceDecel=force_decel),
    "selfdriveState": SimpleNamespace(personality=1),
    "radarState": SimpleNamespace(
      leadOne=lead_one or make_lead(),
      leadTwo=lead_two or make_lead(),
    ),
    "starpilotCarState": SimpleNamespace(
      ecoGear=eco_gear,
      sportGear=sport_gear,
      forceCoast=force_coast,
      pulseAndGlide=pulse_and_glide,
      trafficModeEnabled=traffic_mode,
    ),
  }


def test_slc_coast_window_prefers_coast_for_small_overspeed():
  target = 56.0 * CV.MPH_TO_MS
  accel = StarPilotAcceleration(FakePlanner(v_cruise=target, slc_target=target))
  sm = make_sm(set_speed_kph=100.0)

  accel.update(57.0 * CV.MPH_TO_MS, sm, make_toggles(deceleration_profile=DECELERATION_PROFILES["ECO"]))

  assert accel.min_accel == pytest.approx(-0.02, abs=1e-3)


def test_slc_coast_window_does_not_require_starpilot_plan_message():
  target = 56.0 * CV.MPH_TO_MS
  accel = StarPilotAcceleration(FakePlanner(v_cruise=target, slc_target=target))
  sm = make_sm(set_speed_kph=100.0)

  accel.update(57.0 * CV.MPH_TO_MS, sm, make_toggles(deceleration_profile=DECELERATION_PROFILES["ECO"]))

  assert accel.min_accel == pytest.approx(-0.02, abs=1e-3)


def test_slc_coast_window_uses_effective_target_with_offset_and_cluster_diff():
  raw_target = 58.0 * CV.MPH_TO_MS
  slc_target = 45.0 * CV.MPH_TO_MS
  slc_offset = 3.0 * CV.MPH_TO_MS
  v_ego = 48.0 * CV.MPH_TO_MS
  v_ego_cluster = v_ego + 0.4
  accel = StarPilotAcceleration(FakePlanner(v_cruise=raw_target, slc_target=slc_target, slc_offset=slc_offset))
  sm = make_sm(set_speed_kph=100.0, v_ego_cluster=v_ego_cluster)

  accel.update(v_ego, sm, make_toggles(deceleration_profile=DECELERATION_PROFILES["ECO"]))

  assert accel.min_accel == pytest.approx(-0.02, abs=1e-3)


def test_slc_coast_window_still_applies_when_set_speed_limit_is_off():
  raw_target = 58.0 * CV.MPH_TO_MS
  slc_target = 45.0 * CV.MPH_TO_MS
  slc_offset = 3.0 * CV.MPH_TO_MS
  v_ego = 48.0 * CV.MPH_TO_MS
  accel = StarPilotAcceleration(FakePlanner(v_cruise=raw_target, slc_target=slc_target, slc_offset=slc_offset))
  sm = make_sm(set_speed_kph=100.0, v_ego_cluster=v_ego + 0.4)

  accel.update(v_ego, sm, make_toggles(deceleration_profile=DECELERATION_PROFILES["ECO"], set_speed_limit=False))

  assert accel.min_accel == pytest.approx(-0.02, abs=1e-3)


def test_slc_coast_window_scales_by_profile_strength():
  v_ego = 65.0 * CV.MPH_TO_MS
  v_target = 60.0 * CV.MPH_TO_MS

  eco = get_slc_shaped_min_accel(v_ego, v_target, DECELERATION_PROFILES["ECO"], A_CRUISE_MIN_ECO)
  standard = get_slc_shaped_min_accel(v_ego, v_target, DECELERATION_PROFILES["STANDARD"], A_CRUISE_MIN)
  sport = get_slc_shaped_min_accel(v_ego, v_target, DECELERATION_PROFILES["SPORT"], A_CRUISE_MIN * 2)

  assert eco > standard > sport


def test_slc_coast_window_disabled_for_relevant_lead():
  target = 56.0 * CV.MPH_TO_MS
  accel = StarPilotAcceleration(FakePlanner(v_cruise=target, slc_target=target))
  sm = make_sm(
    set_speed_kph=100.0,
    lead_one=make_lead(status=True, d_rel=20.0, v_lead=45.0 * CV.MPH_TO_MS, a_lead_k=0.0),
  )

  accel.update(57.0 * CV.MPH_TO_MS, sm, make_toggles(deceleration_profile=DECELERATION_PROFILES["ECO"]))

  assert accel.min_accel == pytest.approx(A_CRUISE_MIN_ECO)


def test_slc_coast_window_disabled_when_target_drop_is_not_slc():
  slc_target = 60.0 * CV.MPH_TO_MS
  accel = StarPilotAcceleration(FakePlanner(v_cruise=55.0 * CV.MPH_TO_MS, slc_target=slc_target))
  sm = make_sm(
    set_speed_kph=100.0,
  )

  accel.update(57.0 * CV.MPH_TO_MS, sm, make_toggles(deceleration_profile=DECELERATION_PROFILES["ECO"]))

  assert accel.min_accel == pytest.approx(A_CRUISE_MIN_ECO)


SET_KPH = 100.0
SET_MS = SET_KPH * CV.KPH_TO_MS


def _gas_override(v_ego, *, profile=DECELERATION_PROFILES["STANDARD"], planner_v_cruise=SET_MS, release_sm=None,
                  toggles=None, **planner_kwargs):
  accel = StarPilotAcceleration(FakePlanner(v_cruise=planner_v_cruise, **planner_kwargs))
  toggles = toggles or make_toggles(deceleration_profile=profile, speed_limit_controller=False)
  accel.update(v_ego, make_sm(set_speed_kph=SET_KPH, gas_pressed=True), toggles)
  accel.update(v_ego, release_sm or make_sm(set_speed_kph=SET_KPH), toggles)
  return accel


def test_gas_override_small_overshoot_coasts_at_gas_off_estimate():
  accel = _gas_override(SET_MS + 0.5)
  assert accel.min_accel == pytest.approx(-0.3, abs=1e-3)


def test_gas_override_downhill_keeps_slc_coast_floor():
  accel = _gas_override(SET_MS + 0.5, release_sm=make_sm(set_speed_kph=SET_KPH, pitch=-0.1))
  assert accel.min_accel == pytest.approx(-0.03, abs=1e-3)


def test_gas_override_eco_coast_never_deeper_than_profile():
  accel = _gas_override(SET_MS + 0.5, release_sm=make_sm(set_speed_kph=SET_KPH, pitch=0.1),
                        profile=DECELERATION_PROFILES["ECO"])
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN_ECO)


def test_gas_override_large_overshoot_builds_to_profile_floor():
  floors = [_gas_override(SET_MS + over).min_accel for over in (0.5, 1.5, 2.5, 3.5, 6.0)]
  assert all(a >= b for a, b in zip(floors, floors[1:], strict=False))
  assert floors[1] > -0.4
  assert floors[-1] == pytest.approx(A_CRUISE_MIN)


def test_overshoot_without_gas_press_keeps_profile_floor():
  accel = StarPilotAcceleration(FakePlanner(v_cruise=SET_MS))
  accel.update(SET_MS + 0.5, make_sm(set_speed_kph=SET_KPH), make_toggles(deceleration_profile=DECELERATION_PROFILES["STANDARD"],
                                                                         speed_limit_controller=False))
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)


@pytest.mark.parametrize("hazard", [
  {"lead_one": make_lead(status=True, d_rel=20.0, v_lead=20.0)},
  {"lead_one": make_lead(status=True, d_rel=80.0, v_lead=SET_MS + 0.5, a_lead_k=-1.0)},
  {"force_decel": True},
])
def test_gas_override_coast_off_for_lead_and_forced_decel(hazard):
  accel = _gas_override(SET_MS + 0.5, release_sm=make_sm(set_speed_kph=SET_KPH, **hazard))
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)


@pytest.mark.parametrize("planner_kwargs", [{"red_light": True}, {"forcing_stop": True}, {"disable_throttle": True}])
def test_gas_override_coast_off_for_stop_and_red_light(planner_kwargs):
  accel = _gas_override(SET_MS + 0.5, **planner_kwargs)
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)


def test_gas_override_coast_off_when_curve_target_is_below_set_speed():
  accel = _gas_override(SET_MS + 0.5, planner_v_cruise=SET_MS - 3.0)
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)


def test_gas_override_latch_clears_at_set_speed_and_on_set_speed_change():
  toggles = make_toggles(deceleration_profile=DECELERATION_PROFILES["STANDARD"], speed_limit_controller=False)
  accel = _gas_override(SET_MS + 0.5, toggles=toggles)
  accel.update(SET_MS, make_sm(set_speed_kph=SET_KPH), toggles)
  accel.update(SET_MS + 0.5, make_sm(set_speed_kph=SET_KPH), toggles)
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)

  accel = _gas_override(SET_MS + 0.5, toggles=toggles)
  accel.starpilot_planner.v_cruise = SET_MS - 2.0
  accel.update(SET_MS + 0.5, make_sm(set_speed_kph=SET_KPH - 8.0), toggles)
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)


def test_gas_override_coast_applies_to_mapped_sport_gear():
  toggles = make_toggles(deceleration_profile=DECELERATION_PROFILES["STANDARD"], speed_limit_controller=False,
                         map_deceleration=True)
  accel = StarPilotAcceleration(FakePlanner(v_cruise=SET_MS))
  accel.update(SET_MS + 0.5, make_sm(set_speed_kph=SET_KPH, sport_gear=True, gas_pressed=True), toggles)
  accel.update(SET_MS + 0.5, make_sm(set_speed_kph=SET_KPH, sport_gear=True), toggles)
  assert accel.min_accel == pytest.approx(-0.3, abs=1e-3)
  accel.update(SET_MS + 6.0, make_sm(set_speed_kph=SET_KPH, sport_gear=True), toggles)
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN_SPORT)


def test_gas_override_coast_flag_off_keeps_profile_floor(monkeypatch):
  import openpilot.starpilot.controls.lib.starpilot_acceleration as sa
  monkeypatch.setattr(sa, "GAS_OVERRIDE_COAST", False)
  accel = _gas_override(SET_MS + 0.5)
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)


def test_truck_tuning_standard_profile_keeps_non_binding_launch_headroom():
  assert get_max_accel_standard(0.0, ev_tuning=False, truck_tuning=True) == pytest.approx(6.0)
  assert get_max_accel_standard(5.0, ev_tuning=False, truck_tuning=True) == pytest.approx(1.10)


def test_truck_tuning_standard_profile_uses_proven_cruise_limits():
  assert get_max_accel_standard(15.0, ev_tuning=False, truck_tuning=True) == pytest.approx(0.60)
  assert get_max_accel_standard(25.0, ev_tuning=False, truck_tuning=True) == pytest.approx(0.45)


def test_truck_tuning_standard_profile_limits_highway_run_up():
  assert get_max_accel_standard(40.0, ev_tuning=False, truck_tuning=True) == pytest.approx(0.35)


def test_traffic_curve_anchors():
  assert get_max_accel_traffic(0.0) == pytest.approx(1.10)
  assert get_max_accel_traffic(10.0) == pytest.approx(0.67)
  assert get_max_accel_traffic(25.0) == pytest.approx(0.34)
  assert get_max_accel_traffic(40.0) == pytest.approx(0.23)


def test_traffic_curve_softer_than_eco():
  for v in A_CRUISE_MAX_BP_CUSTOM:
    assert get_max_accel_traffic(v) < get_max_accel_eco(v, ev_tuning=True)
    assert get_max_accel_traffic(v) < get_max_accel_eco(v, ev_tuning=False)


def test_traffic_mode_overrides_custom_accel_profile():
  accel = StarPilotAcceleration(FakePlanner(v_cruise=25.0))
  sm = make_sm(traffic_mode=True)

  accel.update(5.0, sm, make_toggles(custom_accel_profile=True, custom_accel_profile_values=[6.0] * 7))

  assert accel.max_accel == pytest.approx(get_max_accel_traffic(5.0))


def test_custom_accel_profile_uses_configured_breakpoints():
  accel = StarPilotAcceleration(FakePlanner(v_cruise=25.0))
  sm = make_sm()
  breakpoints = [0.0, 10.0, 20.0]

  accel.update(10.0, sm, make_toggles(
    custom_accel_profile=True,
    custom_accel_profile_breakpoints=breakpoints,
    custom_accel_profile_values=[3.0, 1.25, 0.5],
  ))

  assert accel.max_accel == pytest.approx(1.25)


def test_traffic_mode_sets_soft_cruise_decel_floor():
  accel = StarPilotAcceleration(FakePlanner(v_cruise=25.0))
  sm = make_sm(traffic_mode=True)

  accel.update(5.0, sm, make_toggles())

  assert accel.min_accel == pytest.approx(A_CRUISE_MIN_TRAFFIC)


def test_force_coast_wins_over_traffic_mode_decel():
  accel = StarPilotAcceleration(FakePlanner(v_cruise=25.0))
  sm = make_sm(traffic_mode=True, force_coast=True)

  accel.update(5.0, sm, make_toggles())

  assert accel.min_accel == pytest.approx(A_CRUISE_MIN_ECO)


def test_pulse_and_glide_coasts_at_set_speed_then_resumes_below_delta():
  set_speed = 100.0 * CV.KPH_TO_MS
  delta = 10.0 * CV.KPH_TO_MS
  accel = StarPilotAcceleration(FakePlanner(v_cruise=set_speed))
  toggles = make_toggles(
    pulse_glide_speed_delta=delta,
    deceleration_profile=DECELERATION_PROFILES["STANDARD"],
  )

  accel.update(set_speed, make_sm(set_speed_kph=100.0, pulse_and_glide=True), toggles)
  assert accel.pulse_glide_coasting is True
  assert accel.pulse_glide_target == pytest.approx(set_speed - delta)
  assert accel.min_accel == pytest.approx(PULSE_GLIDE_COAST_MIN_ACCEL)

  accel.update((90.0 * CV.KPH_TO_MS) - 0.05, make_sm(set_speed_kph=100.0, pulse_and_glide=True), toggles)
  assert accel.pulse_glide_coasting is False
  assert accel.pulse_glide_target is None
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)

  accel.update(99.8 * CV.KPH_TO_MS, make_sm(set_speed_kph=100.0, pulse_and_glide=True), toggles)
  assert accel.pulse_glide_coasting is True
  assert accel.pulse_glide_target == pytest.approx(set_speed - delta)
  assert accel.min_accel == pytest.approx(PULSE_GLIDE_COAST_MIN_ACCEL)


def test_pulse_and_glide_pauses_on_steep_grade_then_resumes():
  set_speed = 100.0 * CV.KPH_TO_MS
  delta = 10.0 * CV.KPH_TO_MS
  accel = StarPilotAcceleration(FakePlanner(v_cruise=set_speed))
  toggles = make_toggles(pulse_glide_speed_delta=delta)

  accel.update(set_speed, make_sm(set_speed_kph=100.0, pulse_and_glide=True, pitch=math.radians(4.0)), toggles)
  assert accel.pulse_glide_coasting is False
  assert accel.pulse_glide_hill_paused is True

  accel.update(set_speed, make_sm(set_speed_kph=100.0, pulse_and_glide=True, pitch=math.radians(2.0)), toggles)
  assert accel.pulse_glide_coasting is True
  assert accel.pulse_glide_hill_paused is False
  assert accel.min_accel == pytest.approx(PULSE_GLIDE_COAST_MIN_ACCEL)


def test_pulse_and_glide_is_inert_when_disabled():
  accel = StarPilotAcceleration(FakePlanner(v_cruise=100.0 * CV.KPH_TO_MS))
  sm = make_sm(set_speed_kph=100.0, pulse_and_glide=False)

  accel.update(100.0 * CV.KPH_TO_MS, sm, make_toggles(deceleration_profile=DECELERATION_PROFILES["STANDARD"], pulse_glide_speed_delta=10.0))

  assert accel.pulse_glide_coasting is False
  assert accel.pulse_glide_target is None
  assert accel.min_accel == pytest.approx(A_CRUISE_MIN)


def test_human_acceleration_low_set_speed_scales_quarter_to_full():
  assert get_max_accel_low_speeds(2.0, 0.0) == pytest.approx(0.5)
  assert get_max_accel_low_speeds(2.0, 12.5) == pytest.approx(1.0)
  assert get_max_accel_low_speeds(2.0, 25.0) == pytest.approx(2.0)
  assert get_max_accel_low_speeds(2.0, 40.0) == pytest.approx(2.0)


def test_human_acceleration_ramps_off_toward_set_speed():
  assert get_max_accel_ramp_off(2.0, 30.0, 30.0) == pytest.approx(0.0)
  assert get_max_accel_ramp_off(2.0, 30.0, 29.0) == pytest.approx(0.5)
  assert get_max_accel_ramp_off(2.0, 30.0, 25.0) == pytest.approx(2.0)
  assert get_max_accel_ramp_off(2.0, 30.0, 31.0) == pytest.approx(0.0)


@pytest.mark.parametrize("v_cruise,v_ego", [(10.0, 2.0), (10.0, 9.5), (30.0, 10.0), (30.0, 29.0), (30.0, 30.0)])
def test_human_acceleration_always_applies_both_limits_and_leaves_braking(v_cruise, v_ego):
  # Baked in since STATUS 118: the HumanAcceleration toggle is no longer read.
  off = StarPilotAcceleration(FakePlanner(v_cruise=v_cruise))
  off.update(v_ego, make_sm(), make_toggles(human_acceleration=False))
  on = StarPilotAcceleration(FakePlanner(v_cruise=v_cruise))
  on.update(v_ego, make_sm(), make_toggles(human_acceleration=True))

  raw = get_max_accel_standard(v_ego, True, False)
  scaled = get_max_accel_low_speeds(raw, v_cruise)
  expected = min(get_max_accel_ramp_off(scaled, v_cruise, v_ego), scaled)
  for accel in (off, on):
    assert accel.max_accel == pytest.approx(expected)
    assert accel.max_accel <= raw
  assert off.min_accel == pytest.approx(on.min_accel)
