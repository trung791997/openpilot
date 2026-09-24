import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from cereal import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.redneck_cruise import (
  DECREASE_INACTIVE_TIMER,
  GAS_SNAP_INTERVAL_S,
  GAS_SNAP_PRESS_S,
  INCREASE_INACTIVE_TIMER,
  LEAD_INCREASE_INACTIVE_TIMER,
  MANUAL_BUTTON_INACTIVE_TIMER,
  RedneckCruise,
  SEND_BUTTON_DECREASE,
  SEND_BUTTON_INCREASE,
  SEND_BUTTON_NONE,
  get_lead_coast_buffer_ms,
  get_lead_departure_boost_ms,
  select_redneck_target_speed,
  update_gas_release_floor,
  update_launch_state,
  want_gas_snap,
)


ButtonType = car.CarState.ButtonEvent.Type


class TestRedneckCruise(unittest.TestCase):
  def setUp(self):
    self.CP = SimpleNamespace()
    self.FPCP = SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True)
    self.redneck = RedneckCruise(self.CP, self.FPCP)

  def _new_state(self, speed_cluster_mph=20.0, button_events=None):
    return SimpleNamespace(
      cruiseState=SimpleNamespace(speedCluster=speed_cluster_mph * CV.MPH_TO_MS),
      buttonEvents=button_events or [],
    )

  @staticmethod
  def _new_control(override=False, cancel=False, resume=False):
    return SimpleNamespace(
      enabled=True,
      cruiseControl=SimpleNamespace(override=override, cancel=cancel, resume=resume),
    )

  @staticmethod
  def _button_event(button_type, pressed):
    return SimpleNamespace(type=button_type, pressed=pressed)

  def _run_until_active(self, target_mph, speed_cluster_mph=20.0, button_events=None,
                        override=False, cancel=False, resume=False, lead_present=False):
    frames = int(max(INCREASE_INACTIVE_TIMER, DECREASE_INACTIVE_TIMER) / DT_CTRL) + 2
    send_button = SEND_BUTTON_NONE
    v_target = 0
    for _ in range(frames):
      send_button, v_target = self.redneck.run(
        self._new_state(speed_cluster_mph=speed_cluster_mph, button_events=button_events),
        self._new_control(override=override, cancel=cancel, resume=resume),
        target_mph * CV.MPH_TO_MS,
        is_metric=False,
        lead_present=lead_present,
      )
      button_events = None
    return send_button, v_target

  def _frames_until_button(self, target_mph, speed_cluster_mph, lead_present=False):
    frames = int(INCREASE_INACTIVE_TIMER / DT_CTRL) + 4
    for frame in range(frames):
      send_button, _ = self.redneck.run(
        self._new_state(speed_cluster_mph=speed_cluster_mph),
        self._new_control(),
        target_mph * CV.MPH_TO_MS,
        is_metric=False,
        lead_present=lead_present,
      )
      if send_button != SEND_BUTTON_NONE:
        return frame
    return None

  def test_increases_cluster_speed_toward_target(self):
    send_button, v_target = self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0)
    self.assertEqual(SEND_BUTTON_INCREASE, send_button)
    self.assertEqual(25, v_target)

  def test_decreases_cluster_speed_toward_target(self):
    send_button, v_target = self._run_until_active(target_mph=20.0, speed_cluster_mph=25.0)
    self.assertEqual(SEND_BUTTON_DECREASE, send_button)
    self.assertEqual(20, v_target)

  def test_decrease_activates_faster_than_increase(self):
    decrease_frame = self._frames_until_button(target_mph=20.0, speed_cluster_mph=25.0)
    self.redneck = RedneckCruise(self.CP, self.FPCP)
    increase_frame = self._frames_until_button(target_mph=25.0, speed_cluster_mph=20.0)

    self.assertIsNotNone(decrease_frame)
    self.assertIsNotNone(increase_frame)
    self.assertLess(decrease_frame, increase_frame)

  def test_lead_increase_activates_faster_than_free_cruise_increase(self):
    free_cruise_frame = self._frames_until_button(target_mph=25.0, speed_cluster_mph=20.0, lead_present=False)
    self.redneck = RedneckCruise(self.CP, self.FPCP)
    lead_frame = self._frames_until_button(target_mph=25.0, speed_cluster_mph=20.0, lead_present=True)

    self.assertIsNotNone(free_cruise_frame)
    self.assertIsNotNone(lead_frame)
    self.assertLess(lead_frame, free_cruise_frame)
    self.assertLessEqual(lead_frame, int(LEAD_INCREASE_INACTIVE_TIMER / DT_CTRL))

  def test_suppresses_output_during_manual_cruise_button_use(self):
    button_event = self._button_event(ButtonType.accelCruise, True)
    send_button, _ = self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0, button_events=[button_event])
    self.assertEqual(SEND_BUTTON_NONE, send_button)

  def test_missing_button_release_only_suppresses_temporarily(self):
    button_event = self._button_event(ButtonType.accelCruise, True)
    send_button, _ = self.redneck.run(
      self._new_state(speed_cluster_mph=20.0, button_events=[button_event]),
      self._new_control(),
      25.0 * CV.MPH_TO_MS,
      is_metric=False,
    )
    self.assertEqual(SEND_BUTTON_NONE, send_button)

    frames = int((MANUAL_BUTTON_INACTIVE_TIMER + INCREASE_INACTIVE_TIMER) / DT_CTRL) + 4
    for _ in range(frames):
      send_button, _ = self.redneck.run(
        self._new_state(speed_cluster_mph=20.0),
        self._new_control(),
        25.0 * CV.MPH_TO_MS,
        is_metric=False,
      )

    self.assertEqual(SEND_BUTTON_INCREASE, send_button)

  def test_suppresses_output_for_capnp_style_button_events(self):
    button_event = SimpleNamespace(type=SimpleNamespace(raw=int(ButtonType.accelCruise)), pressed=True)
    send_button, _ = self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0, button_events=[button_event])
    self.assertEqual(SEND_BUTTON_NONE, send_button)

  def test_suppresses_output_for_override_cancel_and_resume(self):
    for kwargs in ({"override": True}, {"cancel": True}, {"resume": True}):
      with self.subTest(**kwargs):
        send_button, _ = self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0, **kwargs)
        self.assertEqual(SEND_BUTTON_NONE, send_button)

  def test_resets_when_pcm_cruise_speed_is_enabled(self):
    self.FPCP.pcmCruiseSpeed = True
    send_button, v_target = self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0)
    self.assertEqual(SEND_BUTTON_NONE, send_button)
    self.assertEqual(0, v_target)

  def test_target_speed_returns_internal_max_when_plan_only_wants_to_speed_back_up(self):
    target_speed = select_redneck_target_speed(
      120.0,
      77.0 * CV.MPH_TO_MS,
      0.0,
      [78.3 * CV.MPH_TO_MS, 78.2 * CV.MPH_TO_MS, 78.1 * CV.MPH_TO_MS],
      10,
    )
    self.assertAlmostEqual(120.0 * CV.KPH_TO_MS, target_speed)

  def test_target_speed_ignores_plan_drift_during_free_cruise(self):
    target_speed = select_redneck_target_speed(
      104.4,
      63.0 * CV.MPH_TO_MS,
      0.0,
      [62.55 * CV.MPH_TO_MS, 62.44 * CV.MPH_TO_MS, 62.36 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=False,
    )
    self.assertAlmostEqual(104.4 * CV.KPH_TO_MS, target_speed)

  def test_target_speed_caps_set_speed_at_csc_target(self):
    for slc_mph in (0.0, 70.0):
      with self.subTest(slc_mph=slc_mph):
        target_speed = select_redneck_target_speed(
          65.0 * CV.MPH_TO_KPH,
          65.0 * CV.MPH_TO_MS,
          65.0 * CV.MPH_TO_MS,
          [65.0 * CV.MPH_TO_MS] * 3,
          10,
          allow_plan_decrease=False,
          slc_target_speed_ms=slc_mph * CV.MPH_TO_MS,
          csc_target_speed_ms=40.0 * CV.MPH_TO_MS,
        )
        self.assertAlmostEqual(40.0 * CV.MPH_TO_MS, target_speed)

  def test_target_speed_csc_never_raises_set_speed(self):
    target_speed = select_redneck_target_speed(
      55.0 * CV.MPH_TO_KPH, 55.0 * CV.MPH_TO_MS, 0.0, [], 10,
      csc_target_speed_ms=70.0 * CV.MPH_TO_MS,
    )
    self.assertAlmostEqual(55.0 * CV.MPH_TO_MS, target_speed)

  def test_card_passes_csc_target_only_while_csc_controls(self):
    for controlling, expected_mph in ((True, 40.0), (False, 65.0)):
      with self.subTest(controlling=controlling):
        starpilot_plan = SimpleNamespace(vCruise=40.0 * CV.MPH_TO_MS, cscControllingSpeed=controlling,
                                         cscSpeed=40.0 * CV.MPH_TO_MS)
        sm = MagicMock()
        sm.seen = {"starpilotPlan": True, "longitudinalPlan": False, "radarState": False}
        sm.valid = sm.seen.copy()
        sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan}.__getitem__
        card = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=False), sm=sm,
                               starpilot_toggles=SimpleNamespace(speed_limit_controller=False))
        car_state = SimpleNamespace(vEgo=65.0 * CV.MPH_TO_MS, vCruise=65.0 * CV.MPH_TO_KPH,
                                    cruiseState=SimpleNamespace(speedCluster=65.0 * CV.MPH_TO_MS))
        car_control = SimpleNamespace(actuators=SimpleNamespace(accel=0.0), hudControl=SimpleNamespace(leadVisible=False))

        target_speed, _ = Car._get_redneck_target_speed(card, car_state, car_control)

        self.assertAlmostEqual(expected_mph * CV.MPH_TO_MS, target_speed, places=3)

  def test_target_speed_respects_manual_lower_set_speed_with_slc(self):
    for internal_mph, slc_mph, expected_mph in ((55.0, 65.0, 55.0), (65.0, 55.0, 55.0)):
      with self.subTest(internal_mph=internal_mph, slc_mph=slc_mph):
        target_speed = select_redneck_target_speed(
          internal_mph * CV.MPH_TO_KPH,
          internal_mph * CV.MPH_TO_MS,
          0.0,
          [],
          10,
          allow_plan_decrease=False,
          slc_target_speed_ms=slc_mph * CV.MPH_TO_MS,
        )
        self.assertAlmostEqual(expected_mph * CV.MPH_TO_MS, target_speed)

  def test_target_speed_keeps_slc_limit_when_manual_set_speed_is_higher(self):
    target_speed = select_redneck_target_speed(
      75.0 * CV.MPH_TO_KPH,
      65.0 * CV.MPH_TO_MS,
      0.0,
      [],
      10,
      allow_plan_decrease=False,
      slc_target_speed_ms=65.0 * CV.MPH_TO_MS,
    )

    self.assertAlmostEqual(65.0 * CV.MPH_TO_MS, target_speed)

  def test_card_target_speed_uses_longitudinal_acceleration(self):
    sm = MagicMock()
    sm.seen = {"starpilotPlan": False, "longitudinalPlan": False, "radarState": False}
    sm.valid = sm.seen.copy()
    card = SimpleNamespace(
      CP=SimpleNamespace(openpilotLongitudinalControl=True),
      sm=sm,
      starpilot_toggles=SimpleNamespace(speed_limit_controller=False),
    )
    car_state = SimpleNamespace(vEgo=55.0 * CV.MPH_TO_MS)
    car_control = SimpleNamespace(
      actuators=SimpleNamespace(accel=0.5),
      hudControl=SimpleNamespace(leadVisible=True),
    )

    target_speed, lead_present = Car._get_redneck_target_speed(card, car_state, car_control)

    self.assertAlmostEqual(55.0 * CV.MPH_TO_MS * 1.01 + 1.5, target_speed)
    self.assertTrue(lead_present)

  def test_card_target_speed_uses_slc_target_with_longitudinal_control(self):
    slc_target = 80.0 * CV.KPH_TO_MS
    starpilot_plan = SimpleNamespace(
      vCruise=110.0 * CV.KPH_TO_MS,
      slcOverriddenSpeed=0.0,
      slcSpeedLimit=slc_target,
      slcSpeedLimitOffset=0.0,
    )
    sm = MagicMock()
    sm.seen = {"starpilotPlan": True, "longitudinalPlan": False, "radarState": False}
    sm.valid = sm.seen.copy()
    sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan}.__getitem__
    card = SimpleNamespace(
      CP=SimpleNamespace(openpilotLongitudinalControl=True),
      sm=sm,
      starpilot_toggles=SimpleNamespace(speed_limit_controller=True),
    )
    car_state = SimpleNamespace(
      vEgo=100.0 * CV.KPH_TO_MS,
      cruiseState=SimpleNamespace(speedCluster=70.0 * CV.KPH_TO_MS),
    )
    car_control = SimpleNamespace(
      actuators=SimpleNamespace(accel=-1.0),
      hudControl=SimpleNamespace(leadVisible=False),
    )

    target_speed, lead_present = Car._get_redneck_target_speed(card, car_state, car_control)

    self.assertAlmostEqual(slc_target, target_speed)
    self.assertFalse(lead_present)

  def test_card_target_speed_honors_lower_redneck_slc_override(self):
    starpilot_plan = SimpleNamespace(
      vCruise=65.0 * CV.KPH_TO_MS,
      slcOverriddenSpeed=55.0 * CV.KPH_TO_MS,
      slcSpeedLimit=65.0 * CV.KPH_TO_MS,
      slcSpeedLimitOffset=0.0,
    )
    sm = MagicMock()
    sm.seen = {"starpilotPlan": True, "longitudinalPlan": False, "radarState": False}
    sm.valid = sm.seen.copy()
    sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan}.__getitem__
    card = SimpleNamespace(
      CP=SimpleNamespace(openpilotLongitudinalControl=True),
      sm=sm,
      starpilot_toggles=SimpleNamespace(
        speed_limit_controller=True,
        redneck_cruise=True,
        speed_limit_controller_override_set_speed=True,
      ),
    )
    car_state = SimpleNamespace(
      vEgo=60.0 * CV.KPH_TO_MS,
      vCruise=65.0,
      cruiseState=SimpleNamespace(speedCluster=65.0 * CV.KPH_TO_MS),
    )
    car_control = SimpleNamespace(
      actuators=SimpleNamespace(accel=0.0),
      hudControl=SimpleNamespace(leadVisible=False),
    )

    target_speed, lead_present = Car._get_redneck_target_speed(card, car_state, car_control)

    self.assertAlmostEqual(55.0 * CV.KPH_TO_MS, target_speed)
    self.assertFalse(lead_present)

  def test_target_speed_returns_plan_minimum_when_slowing_down(self):
    target_speed = select_redneck_target_speed(
      120.0,
      75.0 * CV.MPH_TO_MS,
      0.0,
      [74.0 * CV.MPH_TO_MS, 72.0 * CV.MPH_TO_MS, 71.0 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=True,
    )
    self.assertAlmostEqual(71.0 * CV.MPH_TO_MS, target_speed)

  def test_target_speed_uses_longer_horizon_and_buffer_for_lead_slowdown(self):
    target_speed = select_redneck_target_speed(
      120.0,
      75.0 * CV.MPH_TO_MS,
      0.0,
      [74.9 * CV.MPH_TO_MS, 74.6 * CV.MPH_TO_MS, 74.2 * CV.MPH_TO_MS, 73.8 * CV.MPH_TO_MS,
       73.4 * CV.MPH_TO_MS, 73.0 * CV.MPH_TO_MS, 72.6 * CV.MPH_TO_MS, 72.2 * CV.MPH_TO_MS,
       71.8 * CV.MPH_TO_MS, 71.4 * CV.MPH_TO_MS, 71.0 * CV.MPH_TO_MS],
      11,
      allow_plan_decrease=True,
      lead_present=True,
    )
    self.assertLess(target_speed, 71.4 * CV.MPH_TO_MS)

  def test_lead_coast_buffer_grows_with_closing_speed_and_tighter_headway(self):
    base_buffer = get_lead_coast_buffer_ms(
      75.0 * CV.MPH_TO_MS,
      0.0,
      0.0,
    )
    fast_closing_far_buffer = get_lead_coast_buffer_ms(
      75.0 * CV.MPH_TO_MS,
      70.0,
      -5.0 * CV.MPH_TO_MS,
    )
    fast_closing_near_buffer = get_lead_coast_buffer_ms(
      75.0 * CV.MPH_TO_MS,
      35.0,
      -5.0 * CV.MPH_TO_MS,
    )

    self.assertGreater(fast_closing_far_buffer, base_buffer)
    self.assertGreater(fast_closing_near_buffer, fast_closing_far_buffer)

  def test_target_speed_uses_extra_lead_buffer_when_closing_on_slower_car(self):
    target_speed = select_redneck_target_speed(
      120.0,
      56.0 * CV.MPH_TO_MS,
      0.0,
      [55.9 * CV.MPH_TO_MS, 55.7 * CV.MPH_TO_MS, 55.3 * CV.MPH_TO_MS, 55.0 * CV.MPH_TO_MS,
       54.8 * CV.MPH_TO_MS],
      5,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=60.0,
      lead_rel_speed_ms=-4.5 * CV.MPH_TO_MS,
    )
    self.assertLess(target_speed, 53.0 * CV.MPH_TO_MS)

  def test_target_speed_does_not_recover_while_closing_on_lead(self):
    target_speed = select_redneck_target_speed(
      120.0,
      88.0 * CV.KPH_TO_MS,
      0.0,
      [89.0 * CV.KPH_TO_MS, 89.0 * CV.KPH_TO_MS, 88.0 * CV.KPH_TO_MS,
       87.0 * CV.KPH_TO_MS, 85.0 * CV.KPH_TO_MS, 80.0 * CV.KPH_TO_MS],
      6,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=46.8,
      lead_rel_speed_ms=-2.2,
    )

    self.assertLess(target_speed, 80.0 * CV.KPH_TO_MS)

  def test_target_speed_coasts_before_closing_lead_plan_crosses_set_speed(self):
    target_speed = select_redneck_target_speed(
      120.0,
      100.0 * CV.KPH_TO_MS,
      0.0,
      [106.0 * CV.KPH_TO_MS, 105.0 * CV.KPH_TO_MS, 104.0 * CV.KPH_TO_MS,
       103.0 * CV.KPH_TO_MS, 102.0 * CV.KPH_TO_MS],
      5,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=55.8,
      lead_rel_speed_ms=-1.1,
    )

    self.assertLess(target_speed, 100.0 * CV.KPH_TO_MS)

  def test_target_speed_holds_for_distant_closing_lead(self):
    target_speed = select_redneck_target_speed(
      120.0,
      100.0 * CV.KPH_TO_MS,
      0.0,
      [106.0 * CV.KPH_TO_MS, 105.0 * CV.KPH_TO_MS, 104.0 * CV.KPH_TO_MS,
       103.0 * CV.KPH_TO_MS, 102.0 * CV.KPH_TO_MS],
      5,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=150.0,
      lead_rel_speed_ms=-1.1,
    )

    self.assertAlmostEqual(100.0 * CV.KPH_TO_MS, target_speed)

  def test_target_speed_uses_near_term_recovery_for_lead_speedup(self):
    target_speed = select_redneck_target_speed(
      120.0,
      55.0 * CV.MPH_TO_MS,
      0.0,
      [57.15 * CV.MPH_TO_MS, 56.9 * CV.MPH_TO_MS, 56.4 * CV.MPH_TO_MS, 55.8 * CV.MPH_TO_MS,
       54.88 * CV.MPH_TO_MS, 52.0 * CV.MPH_TO_MS, 50.15 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=True,
      lead_present=True,
    )
    self.assertAlmostEqual(55.8 * CV.MPH_TO_MS, target_speed)

  def test_lead_departure_boost_requires_positive_rel_speed_and_stable_plan(self):
    boost = get_lead_departure_boost_ms(
      77.0 * CV.MPH_TO_MS,
      63.0,
      2.0 * CV.MPH_TO_MS,
      [77.4 * CV.MPH_TO_MS, 77.5 * CV.MPH_TO_MS, 77.6 * CV.MPH_TO_MS],
    )
    blocked_by_plan = get_lead_departure_boost_ms(
      77.0 * CV.MPH_TO_MS,
      63.0,
      2.0 * CV.MPH_TO_MS,
      [77.4 * CV.MPH_TO_MS, 76.9 * CV.MPH_TO_MS, 77.6 * CV.MPH_TO_MS],
    )
    blocked_by_headway = get_lead_departure_boost_ms(
      77.0 * CV.MPH_TO_MS,
      35.0,
      2.0 * CV.MPH_TO_MS,
      [77.4 * CV.MPH_TO_MS, 77.5 * CV.MPH_TO_MS, 77.6 * CV.MPH_TO_MS],
    )

    self.assertGreater(boost, 0.0)
    self.assertEqual(blocked_by_plan, 0.0)
    self.assertEqual(blocked_by_headway, 0.0)

  def test_target_speed_gets_small_departure_boost_for_opening_lead(self):
    target_speed = select_redneck_target_speed(
      128.0,
      77.0 * CV.MPH_TO_MS,
      0.0,
      [77.4 * CV.MPH_TO_MS, 77.5 * CV.MPH_TO_MS, 77.6 * CV.MPH_TO_MS, 77.8 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=63.0,
      lead_rel_speed_ms=2.0 * CV.MPH_TO_MS,
    )
    self.assertGreater(target_speed, 77.5 * CV.MPH_TO_MS)

  def test_target_speed_holds_current_step_during_lead_recovery(self):
    target_speed = select_redneck_target_speed(
      128.0,
      79.0 * CV.MPH_TO_MS,
      0.0,
      [78.56 * CV.MPH_TO_MS, 78.56 * CV.MPH_TO_MS, 78.56 * CV.MPH_TO_MS, 78.56 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=True,
      lead_present=True,
    )
    self.assertAlmostEqual(79.0 * CV.MPH_TO_MS, target_speed)

  def test_target_speed_does_not_step_down_on_stale_opening_lead_plan(self):
    target_speed = select_redneck_target_speed(
      128.0,
      79.0 * CV.MPH_TO_MS,
      0.0,
      [77.8 * CV.MPH_TO_MS, 77.7 * CV.MPH_TO_MS, 77.6 * CV.MPH_TO_MS, 77.5 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=63.0,
      lead_rel_speed_ms=2.0 * CV.MPH_TO_MS,
    )

    self.assertAlmostEqual(79.0 * CV.MPH_TO_MS, target_speed)

  def test_target_speed_still_slows_for_large_opening_lead_plan_decrease(self):
    target_speed = select_redneck_target_speed(
      128.0,
      79.0 * CV.MPH_TO_MS,
      0.0,
      [76.0 * CV.MPH_TO_MS, 75.5 * CV.MPH_TO_MS, 75.0 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=63.0,
      lead_rel_speed_ms=2.0 * CV.MPH_TO_MS,
    )

    self.assertLess(target_speed, 79.0 * CV.MPH_TO_MS)

  def test_target_speed_does_not_use_recovery_branch_when_cluster_is_above_internal_max(self):
    target_speed = select_redneck_target_speed(
      45.0,
      46.0 * CV.KPH_TO_MS,
      0.0,
      [46.6 * CV.KPH_TO_MS, 46.4 * CV.KPH_TO_MS, 46.2 * CV.KPH_TO_MS, 46.0 * CV.KPH_TO_MS,
       44.0 * CV.KPH_TO_MS, 42.0 * CV.KPH_TO_MS, 39.0 * CV.KPH_TO_MS],
      7,
      allow_plan_decrease=True,
      lead_present=True,
    )
    self.assertLess(target_speed * CV.MS_TO_KPH, 45.0)

  def test_target_speed_stays_on_lead_target_when_cluster_drops_below_it(self):
    target_speed = select_redneck_target_speed(
      76.9,
      32.9 * CV.MPH_TO_MS,
      47.8 * CV.MPH_TO_MS,
      [37.3 * CV.MPH_TO_MS, 37.2 * CV.MPH_TO_MS, 37.1 * CV.MPH_TO_MS],
      10,
      allow_plan_decrease=True,
      lead_present=True,
    )
    self.assertAlmostEqual(37.1 * CV.MPH_TO_MS, target_speed)


class TestRedneckLaunch(unittest.TestCase):
  TARGET = 50.0 * CV.MPH_TO_MS

  def _step(self, state, v_ego_mph, standstill=False, gas=False, button=False, enabled=True):
    return update_launch_state(state[0], state[1], enabled, v_ego_mph * CV.MPH_TO_MS, standstill, gas, button, self.TARGET)

  def test_never_starts_at_standstill(self):
    state = self._step((False, False), 0.0, standstill=True)
    self.assertEqual((False, True), state)
    self.assertEqual((False, True), self._step(state, 0.0, standstill=True))

  def test_starts_once_moving_after_stop(self):
    state = self._step((False, False), 0.0, standstill=True)
    state = self._step(state, 1.0)
    self.assertFalse(state[0])
    state = self._step(state, 3.0)
    self.assertEqual((True, False), state)

  def test_no_launch_without_prior_stop(self):
    self.assertEqual((False, False), self._step((False, False), 10.0))

  def test_ends_near_target(self):
    self.assertTrue(self._step((True, False), 40.0)[0])
    self.assertFalse(self._step((True, False), 48.5)[0])

  def test_ends_on_stop_button_and_disable(self):
    self.assertEqual((False, True), self._step((True, False), 0.0, standstill=True))
    self.assertEqual((False, False), self._step((True, False), 10.0, button=True))
    self.assertEqual((False, False), self._step((True, False), 10.0, enabled=False))

  def test_gas_at_start_defers_launch(self):
    state = self._step((False, True), 5.0, gas=True)
    self.assertEqual((False, True), state)
    self.assertTrue(self._step(state, 6.0)[0])

  def test_card_raises_set_speed_on_launch_with_lead(self):
    starpilot_plan = SimpleNamespace(vCruise=50.0 * CV.MPH_TO_MS, cscControllingSpeed=False, cscSpeed=0.0)
    card = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=False),
                           starpilot_toggles=SimpleNamespace(speed_limit_controller=False))
    car_control = SimpleNamespace(enabled=True, actuators=SimpleNamespace(accel=0.0), hudControl=SimpleNamespace(leadVisible=True))
    targets = []
    for v_mph, standstill in ((0.0, True), (3.0, False), (8.0, False)):
      v = v_mph * CV.MPH_TO_MS
      longitudinal_plan = SimpleNamespace(speeds=[v] * 10, hasLead=True, shouldStop=False, longitudinalPlanSource="lead0")
      sm = MagicMock()
      sm.seen = {"starpilotPlan": True, "longitudinalPlan": True, "radarState": False}
      sm.valid = sm.seen.copy()
      sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan, "longitudinalPlan": longitudinal_plan}.__getitem__
      card.sm = sm
      car_state = SimpleNamespace(vEgo=v, standstill=standstill, gasPressed=False, buttonEvents=[],
                                  vCruise=50.0 * CV.MPH_TO_KPH, cruiseState=SimpleNamespace(speedCluster=25.0 * CV.MPH_TO_MS))
      targets.append(Car._get_redneck_target_speed(card, car_state, car_control)[0])
    self.assertLess(targets[0], 30.0 * CV.MPH_TO_MS)
    self.assertAlmostEqual(50.0 * CV.MPH_TO_MS, targets[1], places=2)
    self.assertAlmostEqual(50.0 * CV.MPH_TO_MS, targets[2], places=2)


  def test_ends_when_set_raised_and_caught_up_to_lead_target(self):
    # Route 262 3:50-4:26: set speed already at the launch target, car following a 35 mph lead.
    set_ms, hold_ms = 49.7 * CV.MPH_TO_MS, 35.0 * CV.MPH_TO_MS
    step = lambda v, s, h: update_launch_state(True, False, True, v * CV.MPH_TO_MS, False, False, False, self.TARGET,
                                               set_speed_ms=s, hold_target_ms=h)
    self.assertFalse(step(34.0, set_ms, hold_ms)[0])
    self.assertTrue(step(30.0, set_ms, hold_ms)[0])  # still catching up to the lead
    self.assertTrue(step(34.0, 25.0 * CV.MPH_TO_MS, hold_ms)[0])  # set speed not raised yet

  def test_card_launch_ends_once_following_lead(self):
    starpilot_plan = SimpleNamespace(vCruise=50.0 * CV.MPH_TO_MS, cscControllingSpeed=False, cscSpeed=0.0)
    card = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=False),
                           starpilot_toggles=SimpleNamespace(speed_limit_controller=False),
                           redneck_launch_active=True, redneck_was_stopped=False)
    car_control = SimpleNamespace(enabled=True, actuators=SimpleNamespace(accel=0.0), hudControl=SimpleNamespace(leadVisible=True))
    v = 35.0 * CV.MPH_TO_MS
    longitudinal_plan = SimpleNamespace(speeds=[v] * 10, hasLead=True, shouldStop=False, longitudinalPlanSource="lead0")
    sm = MagicMock()
    sm.seen = {"starpilotPlan": True, "longitudinalPlan": True, "radarState": False}
    sm.valid = sm.seen.copy()
    sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan, "longitudinalPlan": longitudinal_plan}.__getitem__
    card.sm = sm
    car_state = SimpleNamespace(vEgo=v, standstill=False, gasPressed=False, buttonEvents=[],
                                vCruise=50.0 * CV.MPH_TO_KPH, cruiseState=SimpleNamespace(speedCluster=49.7 * CV.MPH_TO_MS))
    target = Car._get_redneck_target_speed(card, car_state, car_control)[0]
    self.assertFalse(card.redneck_launch_active)
    self.assertLess(target, 36.0 * CV.MPH_TO_MS)


class TestRedneckGasReleaseFloor(unittest.TestCase):
  SET = 25.0 * CV.MPH_TO_MS

  def _step(self, floor, v_mph, prev=True, gas=False, brake=False, button=False, enabled=True, metric=False):
    return update_gas_release_floor(floor, prev, gas, enabled, v_mph * CV.MPH_TO_MS, False, brake, button, self.SET, metric)

  def test_release_above_set_speed_sets_rounded_floor(self):
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, self._step(0.0, 36.8))
    self.assertAlmostEqual(60.0 * CV.KPH_TO_MS, self._step(0.0, 60.2 * CV.KPH_TO_MPH, metric=True))

  def test_no_floor_without_release_or_near_set_speed(self):
    self.assertEqual(0.0, self._step(0.0, 37.0, prev=False))
    self.assertEqual(0.0, self._step(0.0, 37.0, gas=True))
    self.assertEqual(0.0, self._step(0.0, 25.8))

  def test_floor_holds_then_clears_on_brake_button_disable_stop(self):
    floor = self._step(0.0, 37.0)
    self.assertEqual(floor, self._step(floor, 33.0, prev=False))
    self.assertEqual(0.0, self._step(floor, 33.0, prev=False, brake=True))
    self.assertEqual(0.0, self._step(floor, 33.0, prev=False, button=True))
    self.assertEqual(0.0, self._step(floor, 33.0, prev=False, enabled=False))
    self.assertEqual(0.0, self._step(floor, 0.0, prev=False))

  def _card(self, toggle=True):
    starpilot_plan = SimpleNamespace(vCruise=55.0 * CV.MPH_TO_MS, cscControllingSpeed=False, cscSpeed=0.0)
    card = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=False), is_metric=False,
                           starpilot_toggles=SimpleNamespace(speed_limit_controller=False, set_speed_on_gas_release=toggle))
    return card, starpilot_plan

  def _run(self, card, starpilot_plan, v_mph, gas, set_mph, lead_mph):
    lead = lead_mph * CV.MPH_TO_MS
    longitudinal_plan = SimpleNamespace(speeds=[lead] * 10, hasLead=True, shouldStop=False, longitudinalPlanSource="lead0")
    sm = MagicMock()
    sm.seen = {"starpilotPlan": True, "longitudinalPlan": True, "radarState": False}
    sm.valid = sm.seen.copy()
    sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan, "longitudinalPlan": longitudinal_plan}.__getitem__
    card.sm = sm
    car_state = SimpleNamespace(vEgo=v_mph * CV.MPH_TO_MS, standstill=False, gasPressed=gas, brakePressed=False, buttonEvents=[],
                                vCruise=55.0 * CV.MPH_TO_KPH, cruiseState=SimpleNamespace(speedCluster=set_mph * CV.MPH_TO_MS))
    car_control = SimpleNamespace(enabled=True, actuators=SimpleNamespace(accel=0.0), hudControl=SimpleNamespace(leadVisible=True))
    return Car._get_redneck_target_speed(card, car_state, car_control)[0]

  def test_card_floor_keeps_set_speed_at_release_speed(self):
    # Route 262 1:29: released at 37 mph over a 24.9 mph set speed, lead plan ~32 mph.
    card, plan = self._card()
    self._run(card, plan, 37.0, True, 24.9, 32.0)
    target = self._run(card, plan, 37.0, False, 24.9, 32.0)
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, target, places=2)
    target = self._run(card, plan, 34.0, False, 37.0, 32.0)
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, target, places=2)

  def test_card_floor_off_with_toggle(self):
    card, plan = self._card(toggle=False)
    self._run(card, plan, 37.0, True, 24.9, 32.0)
    self.assertLess(self._run(card, plan, 37.0, False, 24.9, 32.0), 33.0 * CV.MPH_TO_MS)

  def test_card_floor_capped_by_csc(self):
    card, plan = self._card()
    plan.cscControllingSpeed, plan.cscSpeed = True, 30.0 * CV.MPH_TO_MS
    self._run(card, plan, 37.0, True, 24.9, 40.0)
    self.assertLessEqual(self._run(card, plan, 37.0, False, 24.9, 40.0), 30.0 * CV.MPH_TO_MS + 1e-6)

if __name__ == "__main__":
  unittest.main()


class TestRedneckGasSnap(unittest.TestCase):
  SET = 25.0 * CV.MPH_TO_MS
  CAP = 55.0 * CV.MPH_TO_MS

  def _want(self, v_mph, gas=True, button=False, brake=False, enabled=True, cap=CAP):
    return want_gas_snap(enabled, gas, button, brake, v_mph * CV.MPH_TO_MS, self.SET, cap)

  def test_wants_snap_only_under_gas_above_set_and_within_cap(self):
    self.assertTrue(self._want(40.0))
    self.assertFalse(self._want(40.0, gas=False))
    self.assertFalse(self._want(25.8))
    self.assertFalse(self._want(56.0))
    self.assertFalse(self._want(40.0, cap=30.0 * CV.MPH_TO_MS))
    self.assertFalse(self._want(40.0, button=True))
    self.assertFalse(self._want(40.0, brake=True))
    self.assertFalse(self._want(40.0, enabled=False))

  def _redneck_run(self, redneck, gas, gas_snap, set_mph=25.0, cancel=False):
    cs = SimpleNamespace(cruiseState=SimpleNamespace(speedCluster=set_mph * CV.MPH_TO_MS), buttonEvents=[], gasPressed=gas)
    cc = SimpleNamespace(enabled=True, cruiseControl=SimpleNamespace(override=False, cancel=cancel, resume=False))
    return redneck.run(cs, cc, 40.0 * CV.MPH_TO_MS, False, gas_snap=gas_snap)[0]

  def test_redneck_pulses_decel_set_while_gas_held(self):
    redneck = RedneckCruise(SimpleNamespace(), SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True))
    period = int(GAS_SNAP_INTERVAL_S / DT_CTRL)
    buttons = [self._redneck_run(redneck, True, True) for _ in range(2 * period)]
    press = int(GAS_SNAP_PRESS_S / DT_CTRL)
    self.assertEqual([SEND_BUTTON_DECREASE] * press + [SEND_BUTTON_NONE] * (period - press), buttons[:period])
    self.assertEqual(buttons[:period], buttons[period:])

  def test_redneck_never_snaps_without_gas_or_on_cancel(self):
    redneck = RedneckCruise(SimpleNamespace(), SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True))
    self.assertEqual(SEND_BUTTON_NONE, self._redneck_run(redneck, False, True, set_mph=40.0))
    self.assertEqual(SEND_BUTTON_NONE, self._redneck_run(redneck, True, True, cancel=True))
    self.assertEqual(SEND_BUTTON_NONE, self._redneck_run(redneck, True, False))

  def test_snapped_release_sets_floor_near_set_speed(self):
    # After a snap the set speed is already ~vEgo, so the release floor must not need set + 1 mph.
    floor = update_gas_release_floor(0.0, True, False, True, 40.2 * CV.MPH_TO_MS, False, False, False,
                                     40.0 * CV.MPH_TO_MS, False, snapped=True)
    self.assertAlmostEqual(40.0 * CV.MPH_TO_MS, floor)
    self.assertEqual(0.0, update_gas_release_floor(0.0, True, False, True, 40.2 * CV.MPH_TO_MS, False, False, False,
                                                   40.0 * CV.MPH_TO_MS, False))

  def test_card_snaps_under_gas_and_holds_set_after_release(self):
    floor_tests = TestRedneckGasReleaseFloor()
    card, plan = floor_tests._card()
    floor_tests._run(card, plan, 38.0, True, 24.9, 32.0)
    self.assertTrue(card.redneck_gas_snap)
    # Set speed snapped to 38 while the gas was held; release at 38.3 still holds 38 over the 32 mph lead plan.
    floor_tests._run(card, plan, 38.3, True, 38.0, 32.0)
    self.assertFalse(card.redneck_gas_snap)
    target = floor_tests._run(card, plan, 38.3, False, 38.0, 32.0)
    self.assertAlmostEqual(38.0 * CV.MPH_TO_MS, target, places=2)

  def test_card_no_snap_with_toggle_off(self):
    floor_tests = TestRedneckGasReleaseFloor()
    card, plan = floor_tests._card(toggle=False)
    floor_tests._run(card, plan, 38.0, True, 24.9, 32.0)
    self.assertFalse(card.redneck_gas_snap)
