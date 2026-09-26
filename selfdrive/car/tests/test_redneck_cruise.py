import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from cereal import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.redneck_cruise import (
  DECREASE_INACTIVE_TIMER,
  FAR_LEAD_DECEL_MS2,
  FAR_LEAD_MODEL_ONLY_CLOSING_MS,
  FAR_LEAD_MODEL_ONLY_PROB,
  INCREASE_BLOCK_AFTER_LEAD_CHANGE_S,
  INCREASE_BLOCK_CLOSING_MS,
  INCREASE_BLOCK_MODEL_PROB,
  INCREASE_BLOCK_RANGE_M,
  IncreaseBlock,
  LEAD_CHANGE_RANGE_JUMP_M,
  closing_target_in_path,
  get_model_only_far_lead_target_ms,
  lead_changed,
  lead_key,
  FAR_LEAD_UNCORROBORATED_DECEL_MS2,
  GAS_RELEASE_FLOOR_MAX_S,
  INCREASE_AFTER_DECREASE_LOCKOUT_S,
  PRESS_PULSE_S,
  PRESS_SETTLE_S,
  GAS_SNAP_INTERVAL_S,
  GAS_SNAP_PRESS_S,
  INCREASE_INACTIVE_TIMER,
  LEAD_INCREASE_INACTIVE_TIMER,
  MANUAL_BUTTON_HELD_MAX_S,
  MANUAL_BUTTON_INACTIVE_TIMER,
  RedneckCruise,
  SEND_BUTTON_DECREASE,
  SEND_BUTTON_INCREASE,
  SEND_BUTTON_NONE,
  SETTING_BUTTON_INACTIVE_TIMER,
  get_far_lead_target_ms,
  get_lead_coast_buffer_ms,
  get_lead_departure_boost_ms,
  is_far_lead_corroborated,
  is_speed_button_press,
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

    # No release edge ever arrives: ICBM stays quiet for the whole held cap, then resumes.
    frames = int((MANUAL_BUTTON_HELD_MAX_S + MANUAL_BUTTON_INACTIVE_TIMER + INCREASE_INACTIVE_TIMER) / DT_CTRL) + 4
    for _ in range(frames):
      send_button, _ = self.redneck.run(
        self._new_state(speed_cluster_mph=20.0),
        self._new_control(),
        25.0 * CV.MPH_TO_MS,
        is_metric=False,
      )

    self.assertEqual(SEND_BUTTON_INCREASE, send_button)

  def _run_frames(self, frames, button_events=None):
    sent = []
    for _ in range(frames):
      send_button, _ = self.redneck.run(
        self._new_state(speed_cluster_mph=20.0, button_events=button_events),
        self._new_control(),
        25.0 * CV.MPH_TO_MS,
        is_metric=False,
      )
      sent.append(send_button)
      button_events = None
    return sent

  def test_held_physical_button_suppresses_output_for_the_whole_hold(self):
    # Bench routes 264/265: a physical RES+ held ~3-4 s auto-repeats in the ECU; the old 0.5 s
    # window let ICBM press decel against it after 0.5 s. Now: press edge, then 3 s with no
    # release -> SEND_BUTTON_NONE on every frame.
    self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0)
    sent = self._run_frames(int(3.0 / DT_CTRL), button_events=[self._button_event(ButtonType.resumeCruise, True)])
    self.assertEqual({SEND_BUTTON_NONE}, set(sent))

  def test_released_physical_button_suppresses_output_for_inactive_timer_only(self):
    self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0)
    self._run_frames(int(3.0 / DT_CTRL), button_events=[self._button_event(ButtonType.resumeCruise, True)])
    quiet = int(MANUAL_BUTTON_INACTIVE_TIMER / DT_CTRL)
    sent = self._run_frames(quiet + int(INCREASE_INACTIVE_TIMER / DT_CTRL) + 4,
                            button_events=[self._button_event(ButtonType.resumeCruise, False)])
    self.assertEqual({SEND_BUTTON_NONE}, set(sent[:quiet]))
    self.assertEqual(SEND_BUTTON_INCREASE, sent[-1])

  def test_distance_button_press_suppresses_output_while_held(self):
    # Stock-ACC 270 2:36-2:45: gap presses overlapped by ICBM frames failed 6 of 7 times.
    for button in (ButtonType.gapAdjustCruise, ButtonType.lkas):
      with self.subTest(button=button):
        self.redneck = RedneckCruise(self.CP, self.FPCP)
        self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0)
        sent = self._run_frames(int(0.3 / DT_CTRL), button_events=[self._button_event(button, True)])
        self.assertEqual({SEND_BUTTON_NONE}, set(sent))

  def test_distance_button_release_suppresses_output_for_setting_timer(self):
    self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0)
    self._run_frames(int(0.2 / DT_CTRL), button_events=[self._button_event(ButtonType.gapAdjustCruise, True)])
    quiet = int(SETTING_BUTTON_INACTIVE_TIMER / DT_CTRL)
    sent = self._run_frames(quiet + int(INCREASE_INACTIVE_TIMER / DT_CTRL) + 4,
                            button_events=[self._button_event(ButtonType.gapAdjustCruise, False)])
    # longer than the speed-button window: repeat gap presses came ~0.7 s apart on 26f/270
    self.assertGreater(SETTING_BUTTON_INACTIVE_TIMER, MANUAL_BUTTON_INACTIVE_TIMER)
    self.assertEqual({SEND_BUTTON_NONE}, set(sent[:quiet]))
    self.assertEqual(SEND_BUTTON_INCREASE, sent[-1])

  def test_speed_button_press_excludes_distance_and_lkas(self):
    self.assertFalse(is_speed_button_press([self._button_event(ButtonType.gapAdjustCruise, True)]))
    self.assertFalse(is_speed_button_press([self._button_event(ButtonType.lkas, True)]))
    self.assertFalse(is_speed_button_press([self._button_event(ButtonType.accelCruise, False)]))
    self.assertTrue(is_speed_button_press([self._button_event(ButtonType.decelCruise, True)]))
    self.assertTrue(is_speed_button_press([self._button_event(ButtonType.cancel, True)]))
    capnp_style = SimpleNamespace(type=SimpleNamespace(raw=int(ButtonType.gapAdjustCruise)), pressed=True)
    self.assertFalse(is_speed_button_press([capnp_style]))

  def test_held_button_cap_assumes_missed_release(self):
    self._run_until_active(target_mph=25.0, speed_cluster_mph=20.0)
    cap = int(MANUAL_BUTTON_HELD_MAX_S / DT_CTRL)
    sent = self._run_frames(cap + int(INCREASE_INACTIVE_TIMER / DT_CTRL) + 4,
                            button_events=[self._button_event(ButtonType.decelCruise, True)])
    self.assertEqual({SEND_BUTTON_NONE}, set(sent[:cap]))
    self.assertEqual(SEND_BUTTON_INCREASE, sent[-1])

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
       103.0 * CV.KPH_TO_MS, 101.0 * CV.KPH_TO_MS],
      5,
      allow_plan_decrease=True,
      lead_present=True,
      lead_distance_m=55.8,
      lead_rel_speed_ms=-1.1,
    )

    # Plan min 101 km/h minus the 2.0 s headway coast buffer (0.995 m/s) lands 0.05 m/s under the
    # 1.5 mph hold band (LEAD_RECOVERY_HOLD_BUFFER_MS, widened from 0.5 mph in 50e1c1d37), so the
    # coast fires. At 102 km/h it sits inside the band and the cluster speed is held instead.
    self.assertLess(target_speed, 100.0 * CV.KPH_TO_MS)

  def test_far_lead_target_is_stopping_distance_speed(self):
    # 25e 723.0 s: lead 102.7 m, vLead 14.7 m/s. d_min = max(6, 1.5*14.7) = 22.05 m.
    self.assertAlmostEqual(get_far_lead_target_ms(102.7, 14.7), 21.4, delta=0.1)
    # stopped lead 100 m ahead: d_min = 6 m -> sqrt(2*1.5*94)
    self.assertAlmostEqual(get_far_lead_target_ms(100.0, 0.0), 16.8, delta=0.1)
    self.assertEqual(get_far_lead_target_ms(0.0, 10.0), float("inf"))
    self.assertEqual(get_far_lead_target_ms(5.0, 0.0), 0.0)

  def _far_lead_args(self):
    # 25e 723.0 s (replay): cruise 80 km/h, cluster 22.22 m/s, chill plan still at the set speed,
    # vision lead 102.7 m closing 7.5 m/s. HEAD returns the set speed here (hold branch).
    return dict(
      v_cruise_kph=80.0, speed_cluster_ms=22.22, starpilot_target_speed_ms=0.0,
      plan_speeds_ms=[22.2, 22.1, 22.0, 22.0, 22.0], lookahead_points=5,
      allow_plan_decrease=True, lead_present=True, lead_distance_m=102.7, lead_rel_speed_ms=-7.5,
    )

  def test_far_lead_lowers_target_for_closing_lead(self):
    args = self._far_lead_args()
    off = select_redneck_target_speed(**args)
    on = select_redneck_target_speed(**args, lead_speed_ms=14.7)
    self.assertAlmostEqual(off, 22.22, places=3)
    self.assertLess(on, 22.22)
    self.assertAlmostEqual(on, get_far_lead_target_ms(102.7, 14.7), places=6)

  def test_far_lead_none_is_byte_identical(self):
    args = self._far_lead_args()
    self.assertEqual(select_redneck_target_speed(**args), select_redneck_target_speed(**args, lead_speed_ms=None))
    args["lead_rel_speed_ms"] = 0.5  # not closing
    self.assertEqual(select_redneck_target_speed(**args), select_redneck_target_speed(**args, lead_speed_ms=14.7))
    args["lead_present"] = False
    self.assertEqual(select_redneck_target_speed(**args), select_redneck_target_speed(**args, lead_speed_ms=0.0))

  def test_far_lead_never_raises_target(self):
    args = self._far_lead_args()
    args["plan_speeds_ms"] = [15.0] * 5  # plan already wants less than the far target
    self.assertEqual(select_redneck_target_speed(**args), select_redneck_target_speed(**args, lead_speed_ms=14.7))

  def _make_card_with_lead(self, v_cruise_kph, cluster_ms, plan, d_rel, v_rel, v_lead):
    starpilot_plan = SimpleNamespace(vCruise=v_cruise_kph * CV.KPH_TO_MS, cscControllingSpeed=False, cscSpeed=0.0)
    longitudinal_plan = SimpleNamespace(speeds=plan, hasLead=True, shouldStop=False, longitudinalPlanSource="lead0")
    radar_state = SimpleNamespace(leadOne=SimpleNamespace(status=True, dRel=d_rel, vRel=v_rel, vLead=v_lead))
    sm = MagicMock()
    sm.seen = {"starpilotPlan": True, "longitudinalPlan": True, "radarState": True}
    sm.valid = sm.seen.copy()
    sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan, "longitudinalPlan": longitudinal_plan,
                                  "radarState": radar_state}.__getitem__
    card = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=False), sm=sm,
                           starpilot_toggles=SimpleNamespace(speed_limit_controller=False, icbm_far_lead=False))
    card.CS = SimpleNamespace(vEgo=cluster_ms, standstill=False, gasPressed=False, buttonEvents=[],
                              vCruise=v_cruise_kph, cruiseState=SimpleNamespace(speedCluster=cluster_ms))
    card.CC = SimpleNamespace(enabled=True, actuators=SimpleNamespace(accel=0.0), hudControl=SimpleNamespace(leadVisible=True))
    return card

  def test_far_lead_toggle_gates_lead_speed(self):
    # 25e 723.0 s inputs through card.py: the toggle decides whether leadOne.vLead reaches the selector.
    card = self._make_card_with_lead(v_cruise_kph=80.0, cluster_ms=22.22, plan=[22.2, 22.1, 22.0, 22.0, 22.0],
                                     d_rel=102.7, v_rel=-7.5, v_lead=14.7)
    card.starpilot_toggles.icbm_far_lead = False
    off, _ = Car._get_redneck_target_speed(card, card.CS, card.CC)
    card.starpilot_toggles.icbm_far_lead = True
    on, _ = Car._get_redneck_target_speed(card, card.CS, card.CC)
    self.assertAlmostEqual(off, 22.22, places=3)
    self.assertAlmostEqual(on, get_far_lead_target_ms(102.7, 14.7), places=6)

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


class TestHondaClusterTruncation(unittest.TestCase):
  def _cluster(self, kph, brand="honda", metric=False):
    redneck = RedneckCruise(SimpleNamespace(brand=brand), SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True))
    cs = SimpleNamespace(cruiseState=SimpleNamespace(speedCluster=kph * CV.KPH_TO_MS))
    redneck._update_calculations(cs, 0.0, metric)
    return redneck.v_cruise_cluster

  def test_truncated_kph_reads_back_as_the_set_mph(self):
    import math
    for mph in range(25, 91):
      self.assertEqual(mph, self._cluster(math.floor(mph * CV.MPH_TO_KPH)), mph)

  def test_route_263_hunt_values(self):
    self.assertEqual([53, 54, 55], [self._cluster(k) for k in (85, 86, 88)])

  def test_metric_and_other_brands_unchanged(self):
    self.assertEqual(86, self._cluster(86, metric=True))
    self.assertEqual(53, self._cluster(86, brand="hyundai"))


class TestFarLeadCorroboration(unittest.TestCase):
  """STATUS 126: the lower far-lead decel only for a radar-backed or confident vision lead."""

  def test_corroboration_gate(self):
    self.assertTrue(is_far_lead_corroborated(True, 0.0))
    self.assertTrue(is_far_lead_corroborated(False, 0.7))
    self.assertFalse(is_far_lead_corroborated(False, 0.69))

  def test_corroborated_lead_uses_lower_decel(self):
    # stopped lead 100 m ahead: d_min = 6 m -> sqrt(2 * decel * 94)
    self.assertAlmostEqual(get_far_lead_target_ms(100.0, 0.0, corroborated=True), (2 * FAR_LEAD_DECEL_MS2 * 94.0) ** 0.5, places=6)
    self.assertAlmostEqual(get_far_lead_target_ms(100.0, 0.0), (2 * FAR_LEAD_UNCORROBORATED_DECEL_MS2 * 94.0) ** 0.5, places=6)
    self.assertLess(get_far_lead_target_ms(100.0, 0.0, corroborated=True), get_far_lead_target_ms(100.0, 0.0))

  def _card_target(self, radar, model_prob):
    # Route 271 BM4 (29:52.9 - 4.5 s, replay): 41.6 mph set, radar lead 82 m closing 12.5 m/s, chill plan still at cruise.
    t = TestRedneckCruise()
    card = t._make_card_with_lead(v_cruise_kph=80.0, cluster_ms=18.6, plan=[19.9] * 5, d_rel=82.1, v_rel=-12.5, v_lead=6.3)
    lead = card.sm["radarState"].leadOne
    lead.radar, lead.modelProb = radar, model_prob
    card.starpilot_toggles.icbm_far_lead = True
    return Car._get_redneck_target_speed(card, card.CS, card.CC)[0]

  def test_card_passes_corroboration(self):
    lower = get_far_lead_target_ms(82.1, 6.3, corroborated=True)
    self.assertAlmostEqual(self._card_target(True, 0.3), lower, places=6)
    self.assertAlmostEqual(self._card_target(False, 0.77), lower, places=6)
    self.assertAlmostEqual(self._card_target(False, 0.5), get_far_lead_target_ms(82.1, 6.3), places=6)


class TestGasReleaseFloorLimits(unittest.TestCase):
  """STATUS 126: the gas-release floor no longer holds the set speed up against a closing lead, and expires."""

  def _card(self):
    starpilot_plan = SimpleNamespace(vCruise=55.0 * CV.MPH_TO_MS, cscControllingSpeed=False, cscSpeed=0.0)
    card = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=False), is_metric=False,
                           starpilot_toggles=SimpleNamespace(speed_limit_controller=False, set_speed_on_gas_release=True))
    return card, starpilot_plan

  def _run(self, card, starpilot_plan, v_mph, gas, set_mph, plan_mph, v_rel=None, d_rel=90.0):
    plan = plan_mph * CV.MPH_TO_MS
    longitudinal_plan = SimpleNamespace(speeds=[plan] * 10, hasLead=True, shouldStop=False, longitudinalPlanSource="lead0")
    radar_state = SimpleNamespace(leadOne=SimpleNamespace(status=True, dRel=d_rel, vRel=v_rel if v_rel is not None else 0.0,
                                                          vLead=v_mph * CV.MPH_TO_MS, radar=True, modelProb=0.9))
    sm = MagicMock()
    sm.seen = {"starpilotPlan": True, "longitudinalPlan": True, "radarState": v_rel is not None}
    sm.valid = sm.seen.copy()
    sm.__getitem__.side_effect = {"starpilotPlan": starpilot_plan, "longitudinalPlan": longitudinal_plan,
                                  "radarState": radar_state}.__getitem__
    card.sm = sm
    car_state = SimpleNamespace(vEgo=v_mph * CV.MPH_TO_MS, standstill=False, gasPressed=gas, brakePressed=False, buttonEvents=[],
                                vCruise=55.0 * CV.MPH_TO_KPH, cruiseState=SimpleNamespace(speedCluster=set_mph * CV.MPH_TO_MS))
    car_control = SimpleNamespace(enabled=True, actuators=SimpleNamespace(accel=0.0), hudControl=SimpleNamespace(leadVisible=True))
    return Car._get_redneck_target_speed(card, car_state, car_control)[0]

  def _release(self, card, plan):
    self._run(card, plan, 37.0, True, 24.9, 32.0)
    return self._run(card, plan, 37.0, False, 24.9, 32.0)

  def test_closing_lead_clears_floor_for_good(self):
    # 271 BM4: a slower car revealed after a release; the lead-driven drop must not be held at the floor.
    card, plan = self._card()
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, self._release(card, plan), places=2)
    self.assertLess(self._run(card, plan, 37.0, False, 37.0, 25.0, v_rel=-5.0), 30.0 * CV.MPH_TO_MS)
    self.assertEqual(0.0, card.redneck_gas_release_floor)
    # The lead stops closing (or leaves): the floor does not come back until the next release.
    self.assertLess(self._run(card, plan, 30.0, False, 30.0, 25.0, v_rel=0.0), 30.0 * CV.MPH_TO_MS)
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, self._release(card, plan), places=2)

  def test_lead_not_closing_keeps_floor(self):
    card, plan = self._card()
    self._release(card, plan)
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, self._run(card, plan, 37.0, False, 37.0, 32.0, v_rel=0.0), places=2)
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, self._run(card, plan, 37.0, False, 37.0, 32.0, v_rel=1.0), places=2)

  def test_near_closing_lead_keeps_floor(self):
    # Route 262 1:29, the floor's own case: release at 37 mph over a 24.9 set behind a radar lead at 43.6 m
    # (2.6 s) closing 2.2 m/s. Stock ACC follows that lead itself; the floor must hold the release speed.
    card, plan = self._card()
    self._release(card, plan)
    for _ in range(50):
      target = self._run(card, plan, 37.0, False, 30.0, 25.0, v_rel=-2.2, d_rel=43.6)
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, target, places=2)
    self.assertGreater(card.redneck_gas_release_floor, 0.0)

  def test_floor_expires(self):
    card, plan = self._card()
    self._release(card, plan)
    frames = int(GAS_RELEASE_FLOOR_MAX_S / DT_CTRL)
    for _ in range(frames - 2):
      target = self._run(card, plan, 37.0, False, 37.0, 32.0)
    self.assertAlmostEqual(37.0 * CV.MPH_TO_MS, target, places=2)
    for _ in range(3):
      target = self._run(card, plan, 37.0, False, 37.0, 32.0)
    self.assertLess(target, 37.0 * CV.MPH_TO_MS)
    self.assertEqual(0.0, card.redneck_gas_release_floor)


class TestHoldAtTruncatedCluster(unittest.TestCase):
  def test_target_equal_to_cluster_holds(self):
    # Route 271 seg28+36.7: 49 mph shows as 78 km/h = 48.47 mph. The hold branch returns that value; it used to
    # round to 48 against a corrected cluster of 49 and press DECEL.
    redneck = RedneckCruise(SimpleNamespace(brand="honda"), SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True))
    cluster_ms = 78 * CV.KPH_TO_MS
    cs = SimpleNamespace(cruiseState=SimpleNamespace(speedCluster=cluster_ms), buttonEvents=[], gasPressed=False)
    cc = SimpleNamespace(enabled=True, cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False))
    buttons = [redneck.run(cs, cc, cluster_ms, False)[0] for _ in range(50)]
    self.assertEqual([SEND_BUTTON_NONE] * 50, buttons)
    self.assertEqual(49, redneck.v_target)
    # One mph below the held value is still a decrease.
    self.assertEqual(SEND_BUTTON_DECREASE, [redneck.run(cs, cc, 48.0 * CV.MPH_TO_MS, False)[0] for _ in range(20)][-1])


class TestPressPacing(unittest.TestCase):
  """STATUS 126: with counter sync, the last step is settle-then-pulse and an increase waits after a decrease."""
  CC = SimpleNamespace(enabled=True, cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False))

  def setUp(self):
    self.redneck = RedneckCruise(SimpleNamespace(), SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True))

  def _buttons(self, target_mph, cluster_mph, seconds, counter_sync=True):
    cs = SimpleNamespace(cruiseState=SimpleNamespace(speedCluster=cluster_mph * CV.MPH_TO_MS), buttonEvents=[], gasPressed=False)
    return [self.redneck.run(cs, self.CC, target_mph * CV.MPH_TO_MS, False, counter_sync=counter_sync)[0]
            for _ in range(int(seconds / DT_CTRL))]

  @staticmethod
  def _runs(buttons, button):
    runs, gaps, n, gap = [], [], 0, 0
    for b in buttons:
      if b == button:
        if n == 0 and runs:
          gaps.append(gap)
        n += 1
        gap = 0
      else:
        if n:
          runs.append(n)
        n = 0
        gap += 1
    if n:
      runs.append(n)
    return runs, gaps

  def test_last_step_is_a_pulse_then_settle(self):
    runs, gaps = self._runs(self._buttons(49.0, 50.0, 2.0), SEND_BUTTON_DECREASE)
    self.assertGreaterEqual(len(runs), 2)
    self.assertTrue(all(r == int(PRESS_PULSE_S / DT_CTRL) for r in runs), runs)
    self.assertTrue(all(g >= int(PRESS_SETTLE_S / DT_CTRL) for g in gaps), gaps)

  def test_without_counter_sync_unchanged(self):
    buttons = self._buttons(49.0, 50.0, 1.0, counter_sync=False)
    runs, _ = self._runs(buttons, SEND_BUTTON_DECREASE)
    self.assertEqual(1, len(runs))
    self.assertEqual(SEND_BUTTON_DECREASE, buttons[-1])

  def test_large_drop_not_delayed(self):
    synced = self._buttons(30.0, 50.0, 1.0)
    self.redneck = RedneckCruise(SimpleNamespace(), SimpleNamespace(pcmCruiseSpeed=False, redneckCruiseAvailable=True))
    legacy = self._buttons(30.0, 50.0, 1.0, counter_sync=False)
    self.assertEqual(synced, legacy)
    self.assertEqual(SEND_BUTTON_DECREASE, synced[-1])

  def test_increase_waits_after_decrease_but_decrease_never_waits(self):
    self._buttons(40.0, 50.0, 0.5)  # decreasing
    buttons = self._buttons(55.0, 45.0, 2.0)
    first_increase = buttons.index(SEND_BUTTON_INCREASE)
    self.assertGreaterEqual(first_increase, int(INCREASE_AFTER_DECREASE_LOCKOUT_S / DT_CTRL) - 1)
    buttons = self._buttons(40.0, 50.0, 0.5)
    self.assertLessEqual(buttons.index(SEND_BUTTON_DECREASE), int(DECREASE_INACTIVE_TIMER / DT_CTRL) + 1)


def _lead(status=True, d_rel=60.0, v_rel=0.0, v_lead=20.0, radar=True, track_id=5, model_prob=0.9, y_rel=0.0):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel, vLead=v_lead, radar=radar, radarTrackId=track_id,
                         modelProb=model_prob, yRel=y_rel)


def _model_lead(prob, x, v, y=0.0):
  return SimpleNamespace(prob=prob, x=[x], v=[v], y=[y])


class TestIncreaseBlock(unittest.TestCase):
  """Route 276 14:36.7 (radar lead left the path, slower car revealed on a curve) and 17:22.5 (radar 70 m <->
  vision 95 m flips)."""

  def test_lead_changed(self):
    radar5 = lead_key(_lead())
    self.assertIsNone(lead_key(_lead(status=False)))
    self.assertFalse(lead_changed(None, radar5))  # a lead appearing can only lower the target
    self.assertTrue(lead_changed(radar5, None))  # lost
    self.assertTrue(lead_changed(radar5, lead_key(_lead(track_id=6))))
    self.assertTrue(lead_changed(radar5, lead_key(_lead(radar=False, track_id=-1))))  # radar -> vision
    self.assertTrue(lead_changed(lead_key(_lead(d_rel=70.0)), lead_key(_lead(d_rel=70.0 + LEAD_CHANGE_RANGE_JUMP_M + 1.0))))
    self.assertFalse(lead_changed(radar5, lead_key(_lead(d_rel=62.0))))

  def test_closing_target_in_path(self):
    v_ego = 18.0
    self.assertTrue(closing_target_in_path((_lead(d_rel=98.8, v_rel=-5.2), None), None, v_ego))
    self.assertFalse(closing_target_in_path((_lead(d_rel=INCREASE_BLOCK_RANGE_M + 5.0, v_rel=-9.0), None), None, v_ego))
    self.assertFalse(closing_target_in_path((_lead(d_rel=80.0, v_rel=-INCREASE_BLOCK_CLOSING_MS + 0.5), None), None, v_ego))
    self.assertFalse(closing_target_in_path((_lead(status=False, d_rel=80.0, v_rel=-9.0), None), None, v_ego))
    # leadTwo counts too
    self.assertTrue(closing_target_in_path((_lead(status=False), _lead(d_rel=90.0, v_rel=-6.0)), None, v_ego))
    # The model's lead below radard's publish threshold, at 14:36.7-like geometry: y +8 on a curve is still in path
    # (no raw-y gate), p 0.26, 102.5 m, 26 mph against 40 mph.
    model = [_model_lead(0.26, 102.5, 26.0 * CV.MPH_TO_MS, y=-8.0)]
    self.assertTrue(closing_target_in_path((None, None), model, 40.2 * CV.MPH_TO_MS))
    self.assertFalse(closing_target_in_path((None, None), [_model_lead(INCREASE_BLOCK_MODEL_PROB - 0.05, 102.5, 11.6)], 18.0))
    self.assertFalse(closing_target_in_path((None, None), [_model_lead(0.9, 130.0, 5.0)], 18.0))

  def test_hold_after_change_then_release(self):
    block = IncreaseBlock()
    self.assertFalse(block.update(_lead(), True, False))
    self.assertTrue(block.update(None, False, False))  # lead lost
    frames = int(INCREASE_BLOCK_AFTER_LEAD_CHANGE_S / DT_CTRL)
    held = [block.update(None, False, False) for _ in range(frames + 5)]
    self.assertTrue(all(held[:frames - 1]))
    self.assertFalse(held[-1])
    self.assertTrue(block.update(None, False, True))  # closing target blocks on its own

  def test_apply_never_lowers_or_delays_a_decrease(self):
    self.assertEqual(IncreaseBlock.apply(20.0, 18.0, True), 18.0)
    self.assertEqual(IncreaseBlock.apply(15.0, 18.0, True), 15.0)
    self.assertEqual(IncreaseBlock.apply(20.0, 18.0, False), 20.0)
    self.assertEqual(IncreaseBlock.apply(20.0, 0.0, True), 20.0)

  @staticmethod
  def _card(lead_one, has_lead, cluster_ms, model=None, v_cruise_kph=80.0, plan=None):
    starpilot_plan = SimpleNamespace(vCruise=v_cruise_kph * CV.KPH_TO_MS, cscControllingSpeed=False, cscSpeed=0.0)
    plan = plan or [cluster_ms + 3.0] * 5
    longitudinal_plan = SimpleNamespace(speeds=plan, hasLead=has_lead, shouldStop=False,
                                        longitudinalPlanSource="lead0" if has_lead else "cruise")
    msgs = {"starpilotPlan": starpilot_plan, "longitudinalPlan": longitudinal_plan,
            "radarState": SimpleNamespace(leadOne=lead_one, leadTwo=_lead(status=False))}
    if model is not None:
      msgs["modelV2"] = SimpleNamespace(leadsV3=model, velocity=SimpleNamespace(x=[cluster_ms]))
    sm = MagicMock()
    sm.seen = dict.fromkeys(("starpilotPlan", "longitudinalPlan", "radarState"), True)
    sm.seen["modelV2"] = model is not None
    sm.valid = sm.seen.copy()
    sm.__getitem__.side_effect = msgs.__getitem__
    card = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=False), sm=sm,
                           starpilot_toggles=SimpleNamespace(speed_limit_controller=False, icbm_far_lead=True))
    CS = SimpleNamespace(vEgo=cluster_ms, standstill=False, gasPressed=False, buttonEvents=[], vCruise=v_cruise_kph,
                         cruiseState=SimpleNamespace(speedCluster=cluster_ms))
    CC = SimpleNamespace(enabled=True, actuators=SimpleNamespace(accel=0.0), hudControl=SimpleNamespace(leadVisible=has_lead))
    return card, CS, CC

  def test_card_no_increase_after_lead_leaves(self):
    # 276 14:36.7: following radar tid 5 at 51.6 m, set 37.9 mph; the lead leaves and the plan goes back to cruise.
    cluster = 37.9 * CV.MPH_TO_MS
    card, CS, CC = self._card(_lead(d_rel=51.6, v_rel=-3.6, v_lead=15.6), True, cluster, plan=[cluster] * 5)
    Car._get_redneck_target_speed(card, CS, CC)
    gone, CS, CC = self._card(_lead(status=False), False, cluster)
    gone.redneck_increase_block = card.redneck_increase_block
    first, _ = Car._get_redneck_target_speed(gone, CS, CC)
    self.assertAlmostEqual(first, cluster, places=6)
    for _ in range(int(INCREASE_BLOCK_AFTER_LEAD_CHANGE_S / DT_CTRL) + 1):
      target, _ = Car._get_redneck_target_speed(gone, CS, CC)
    self.assertGreater(target, cluster + 1.0)
    # With the toggle off (non-Honda) nothing is held.
    card, CS, CC = self._card(_lead(status=False), False, cluster)
    card.starpilot_toggles.icbm_far_lead = False
    card.redneck_increase_block = gone.redneck_increase_block
    self.assertGreater(Car._get_redneck_target_speed(card, CS, CC)[0], cluster + 1.0)

  def test_card_model_lead_closing_blocks_increase(self):
    cluster = 37.9 * CV.MPH_TO_MS
    model = [_model_lead(0.26, 102.5, 26.0 * CV.MPH_TO_MS, y=-8.0)]
    card, CS, CC = self._card(_lead(status=False), False, cluster, model=model)
    self.assertAlmostEqual(Car._get_redneck_target_speed(card, CS, CC)[0], cluster, places=6)
    card, CS, CC = self._card(_lead(status=False), False, cluster, model=[_model_lead(0.05, 102.5, 11.6)])
    self.assertGreater(Car._get_redneck_target_speed(card, CS, CC)[0], cluster + 1.0)

  def test_card_decrease_not_delayed_by_block(self):
    cluster = 45.0 * CV.MPH_TO_MS
    card, CS, CC = self._card(_lead(d_rel=60.0, v_rel=-8.0, v_lead=12.0), True, cluster, plan=[14.0] * 5)
    held = IncreaseBlock()
    held.frames = 1000
    card.redneck_increase_block = held
    target, _ = Car._get_redneck_target_speed(card, CS, CC)
    self.assertLess(target, 14.0 + 0.01)


class TestModelOnlyFarLead(unittest.TestCase):
  """Route 276 8:27.9: vision-only lead from 114 m slowing to 3 mph, plan.hasLead False."""

  def test_gate(self):
    lead = _lead(d_rel=100.0, v_rel=-10.0, v_lead=3.0, radar=False, track_id=-1, model_prob=0.6)
    self.assertAlmostEqual(get_model_only_far_lead_target_ms(lead), get_far_lead_target_ms(100.0, 3.0, corroborated=False))
    self.assertEqual(get_model_only_far_lead_target_ms(_lead(d_rel=100.0, v_rel=-10.0, v_lead=3.0)), float("inf"))  # radar
    for kw in ({"model_prob": FAR_LEAD_MODEL_ONLY_PROB - 0.05}, {"v_rel": -FAR_LEAD_MODEL_ONLY_CLOSING_MS + 0.5},
               {"status": False}):
      args = dict(d_rel=100.0, v_rel=-10.0, v_lead=3.0, radar=False, track_id=-1, model_prob=0.6) | kw
      self.assertEqual(get_model_only_far_lead_target_ms(_lead(**args)), float("inf"), kw)
    self.assertEqual(get_model_only_far_lead_target_ms(None), float("inf"))

  def test_card_uses_it_without_plan_lead(self):
    cluster = 45.0 * CV.MPH_TO_MS
    lead = _lead(d_rel=80.0, v_rel=-12.0, v_lead=8.0, radar=False, track_id=-1, model_prob=0.6)
    card, CS, CC = TestIncreaseBlock._card(lead, False, cluster, plan=[cluster] * 5)
    target, _ = Car._get_redneck_target_speed(card, CS, CC)
    self.assertAlmostEqual(target, get_far_lead_target_ms(80.0, 8.0, corroborated=False), places=6)
    self.assertLess(target, cluster)
