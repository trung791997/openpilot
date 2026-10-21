from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.starpilot.common.experimental_state import CCStatus
from openpilot.starpilot.controls.lib.conditional_chill_mode import ConditionalChillMode


class FakeParams:
  def __init__(self, bools=None, ints=None):
    self.bools = dict(bools or {})
    self.ints = dict(ints or {})

  def get_bool(self, key):
    return bool(self.bools.get(key, False))

  def put_bool(self, key, value):
    self.bools[key] = bool(value)

  def get_int(self, key, default=0):
    return int(self.ints.get(key, default))

  def put_int(self, key, value):
    self.ints[key] = int(value)


class FakeDetector:
  def __init__(self):
    self.curve_detected = False
    self.slow_lead_detected = False
    self.stop_light_detected = False
    self.stop_light_model_detected = False
    self.toggle_objects = []

  def curve_detection(self, _v_ego, toggles):
    self.toggle_objects.append(toggles)
    return None

  def slow_lead(self, toggles, _v_ego):
    self.toggle_objects.append(toggles)
    return None

  def stop_sign_and_light(self, *_args, **_kwargs):
    return None


class FakeSubMaster:
  def __init__(self, services):
    self.services = dict(services)

  def __getitem__(self, key):
    return self.services[key]


def make_sm():
  return {
    "carState": SimpleNamespace(standstill=False, leftBlinker=False, rightBlinker=False),
    "selfdriveState": SimpleNamespace(enabled=True),
    "longitudinalPlan": SimpleNamespace(allowThrottle=True, shouldStop=False),
    "starpilotCarState": SimpleNamespace(trafficModeEnabled=False),
    "starpilotPlan": SimpleNamespace(redLight=False, forcingStop=False),
    "starpilotRadarState": SimpleNamespace(
      leadLeft=SimpleNamespace(status=False, dRel=float("inf"), vLead=0.0),
      leadRight=SimpleNamespace(status=False, dRel=float("inf"), vLead=0.0),
    ),
  }


def make_submaster_sm():
  return FakeSubMaster(make_sm())


def make_toggles():
  return SimpleNamespace(
    conditional_chill_speed=45 * CV.MPH_TO_MS,
    conditional_chill_speed_lead=35 * CV.MPH_TO_MS,
    conditional_chill_speed_margin=3 * CV.MPH_TO_MS,
    conditional_chill_lead=True,
    conditional_chill_launch_assist=False,
  )


def make_ccm():
  planner = SimpleNamespace(
    params=FakeParams(),
    params_memory=FakeParams(),
    starpilot_vcruise=SimpleNamespace(
      stop_sign_confirmed=False,
      forcing_stop=False,
      slc=SimpleNamespace(experimental_mode=False),
    ),
    raw_model_stopped=False,
    model_stopped=False,
    tracking_lead=False,
    lead_one=SimpleNamespace(
      status=False,
      dRel=float("inf"),
      vLead=0.0,
      vRel=0.0,
      aLeadK=0.0,
      modelProb=0.0,
      radar=False,
    ),
  )
  detector = FakeDetector()
  return planner, detector, ConditionalChillMode(planner, detector)


def test_ccm_stays_experimental_when_no_chill_condition_matches(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()

  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: 1.0)
  ccm.update(20 * CV.MPH_TO_MS, 21 * CV.MPH_TO_MS, sm, toggles)

  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_reuses_detector_toggles(monkeypatch):
  _planner, detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: 1.0)

  ccm.update(20 * CV.MPH_TO_MS, 21 * CV.MPH_TO_MS, sm, toggles)
  first_curve_toggles, first_slow_lead_toggles = detector.toggle_objects
  detector.toggle_objects.clear()
  ccm.update(20 * CV.MPH_TO_MS, 21 * CV.MPH_TO_MS, sm, toggles)
  second_curve_toggles, second_slow_lead_toggles = detector.toggle_objects

  assert first_curve_toggles is first_slow_lead_toggles
  assert second_curve_toggles is second_slow_lead_toggles
  assert first_curve_toggles is second_curve_toggles


def test_ccm_enters_chill_for_open_road_speed_recovery(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  monotonic_values = iter([10.0, 10.5])
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: next(monotonic_values))

  v_ego = 55 * CV.MPH_TO_MS
  v_cruise = v_ego + 5 * CV.MPH_TO_MS
  ccm.update(v_ego, v_cruise, sm, toggles)
  ccm.update(v_ego, v_cruise, sm, toggles)

  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["SPEED"]
  assert planner.params_memory.get_int("CCStatus") == CCStatus["SPEED"]


def test_ccm_enters_chill_for_stable_lead_cruising(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  monotonic_values = iter([10.0, 11.1])
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: next(monotonic_values))

  planner.tracking_lead = True
  planner.lead_one.status = True
  planner.lead_one.dRel = 45.0
  planner.lead_one.vLead = 24.8
  planner.lead_one.radar = True

  v_ego = 58 * CV.MPH_TO_MS
  ccm.update(v_ego, v_ego, sm, toggles)
  ccm.update(v_ego, v_ego, sm, toggles)

  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["LEAD"]


def test_ccm_stable_lead_requires_longer_entry_debounce(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  monotonic_values = iter([10.0, 10.6])
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: next(monotonic_values))

  planner.tracking_lead = True
  planner.lead_one.status = True
  planner.lead_one.dRel = 45.0
  planner.lead_one.vLead = 24.8
  planner.lead_one.radar = True

  v_ego = 58 * CV.MPH_TO_MS
  ccm.update(v_ego, v_ego, sm, toggles)
  ccm.update(v_ego, v_ego, sm, toggles)

  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_hard_vetoes_force_experimental(monkeypatch):
  planner, detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  v_ego = 60 * CV.MPH_TO_MS
  v_cruise = v_ego + 5 * CV.MPH_TO_MS

  veto_scenes = []

  detector.slow_lead_detected = True
  veto_scenes.append(("slow_lead", make_sm()))
  detector.slow_lead_detected = False

  traffic_sm = make_sm()
  traffic_sm["starpilotCarState"].trafficModeEnabled = True
  veto_scenes.append(("traffic_mode", traffic_sm))

  adjacent_sm = make_sm()
  adjacent_sm["starpilotRadarState"].leadLeft = SimpleNamespace(status=True, dRel=25.0, vLead=12.0)
  veto_scenes.append(("adjacent_lead", adjacent_sm))

  for index, (_name, scene_sm) in enumerate(veto_scenes, start=1):
    monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda idx=index: float(idx))
    if _name == "slow_lead":
      detector.slow_lead_detected = True
    else:
      detector.slow_lead_detected = False
    ccm.update(v_ego, v_cruise, scene_sm, toggles)
    assert ccm.experimental_mode
    assert ccm.status_value == CCStatus["OFF"]

  planner.starpilot_vcruise.slc.experimental_mode = True
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: 10.0)
  ccm.update(v_ego, v_cruise, sm, toggles)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_far_lateral_adjacent_lead_does_not_block_open_road_chill(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  monotonic_values = iter([10.0, 10.5])
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: next(monotonic_values))

  sm["starpilotRadarState"].leadLeft = SimpleNamespace(status=True, dRel=18.0, vLead=12.0, yRel=6.5)

  v_ego = 55 * CV.MPH_TO_MS
  v_cruise = v_ego + 5 * CV.MPH_TO_MS
  ccm.update(v_ego, v_cruise, sm, toggles)
  ccm.update(v_ego, v_cruise, sm, toggles)

  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["SPEED"]


def test_ccm_immediate_adjacent_lead_still_blocks_open_road_chill(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()

  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: 10.0)
  sm["starpilotRadarState"].leadLeft = SimpleNamespace(status=True, dRel=18.0, vLead=12.0, yRel=4.5)

  v_ego = 55 * CV.MPH_TO_MS
  v_cruise = v_ego + 5 * CV.MPH_TO_MS
  ccm.update(v_ego, v_cruise, sm, toggles)

  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_immediately_exits_chill_when_scene_turns_into_slow_lead(monkeypatch):
  planner, detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  monotonic_values = iter([10.0, 11.1, 11.2])
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: next(monotonic_values))

  planner.tracking_lead = True
  planner.lead_one.status = True
  planner.lead_one.dRel = 40.0
  planner.lead_one.vLead = 24.9
  planner.lead_one.radar = True

  v_ego = 58 * CV.MPH_TO_MS
  ccm.update(v_ego, v_ego, sm, toggles)
  ccm.update(v_ego, v_ego, sm, toggles)
  assert not ccm.experimental_mode

  detector.slow_lead_detected = True
  ccm.update(v_ego, v_ego, sm, toggles)

  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


class Clock:
  def __init__(self, t):
    self.t = t

  def __call__(self):
    return self.t


def launch_ccm(monkeypatch, t=20.0):
  planner, detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  toggles.conditional_chill_launch_assist = True
  clock = Clock(t)
  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", clock)
  return planner, detector, ccm, sm, toggles, clock


def stopped_behind_red_light_lead(planner, detector, sm):
  sm["carState"].standstill = True
  sm["longitudinalPlan"].shouldStop = True
  sm["starpilotPlan"].redLight = True
  detector.stop_light_detected = True
  detector.slow_lead_detected = True
  planner.model_stopped = True
  planner.tracking_lead = True
  planner.lead_one.status = True
  planner.lead_one.dRel = 6.1
  planner.lead_one.vLead = 0.0
  planner.lead_one.vRel = 0.0
  planner.lead_one.radar = True
  sm["starpilotRadarState"].leadLeft = SimpleNamespace(status=True, dRel=8.0, vLead=3.0, yRel=3.5)


def drive(ccm, sm, toggles, clock, v_ego, dt=0.05, steps=1):
  sm["carState"].standstill = v_ego < 0.01
  for _ in range(steps):
    clock.t += dt
    ccm.update(v_ego, 25 * CV.MPH_TO_MS, sm, toggles)


def test_ccm_launch_assist_stays_experimental_while_stopped(monkeypatch):
  planner, detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  sm["carState"].standstill = True
  drive(ccm, sm, toggles, clock, 0.0, steps=40)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]

  stopped_behind_red_light_lead(planner, detector, sm)
  drive(ccm, sm, toggles, clock, 0.0, steps=40)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_launch_assist_enters_chill_once_rolling_with_no_lead(monkeypatch):
  _planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  drive(ccm, sm, toggles, clock, 1.4)
  assert ccm.experimental_mode

  drive(ccm, sm, toggles, clock, 1.6)
  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["SPEED"]


def test_ccm_launch_assist_queue_creep_stays_experimental(monkeypatch):
  # STATUS 261: 3a3 queue creeps stopped again ~1.2 s / 0.6 m after 0.5 m/s.
  planner, detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  stopped_behind_red_light_lead(planner, detector, sm)
  for _ in range(3):
    drive(ccm, sm, toggles, clock, 0.0, steps=20)
    for v in (0.4, 0.8, 1.0, 0.8, 0.4, 0.2):
      drive(ccm, sm, toggles, clock, v, steps=4)
      assert ccm.experimental_mode, v
      assert ccm.status_value == CCStatus["OFF"], v


def test_ccm_launch_assist_enters_chill_behind_lead_at_light_with_stale_stop_flags(monkeypatch):
  planner, detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  stopped_behind_red_light_lead(planner, detector, sm)
  drive(ccm, sm, toggles, clock, 0.0, steps=20)

  # Light turns green, the lead pulls off; some flags lag behind the scene.
  sm["starpilotPlan"].redLight = False
  sm["longitudinalPlan"].shouldStop = False
  planner.lead_one.vLead = 2.0
  drive(ccm, sm, toggles, clock, 1.6)

  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["LEAD"]

  # Stale flags, and a flag clearing then coming back, do not end the launch inside the intersection.
  for v in (2.0, 3.0, 4.0):
    detector.stop_light_detected = v != 3.0
    drive(ccm, sm, toggles, clock, v, steps=4)
    assert not ccm.experimental_mode
    assert ccm.status_value == CCStatus["LEAD"]


def test_ccm_launch_assist_ignores_red_light_coming_back_on_after_green(monkeypatch):
  # STATUS 261: 3a7 25 s / 90 s, redLight rose again at 5.7 m and 7.1 m pulling away on green.
  _planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  sm["starpilotPlan"].redLight = True
  drive(ccm, sm, toggles, clock, 0.0, steps=20)
  sm["starpilotPlan"].redLight = False
  drive(ccm, sm, toggles, clock, 1.6)
  drive(ccm, sm, toggles, clock, 3.0, dt=0.1, steps=19)
  assert not ccm.experimental_mode

  sm["starpilotPlan"].redLight = True
  drive(ccm, sm, toggles, clock, 3.0, dt=0.1, steps=10)
  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["SPEED"]


def test_ccm_launch_assist_enters_chill_at_stop_sign(monkeypatch):
  planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  planner.starpilot_vcruise.stop_sign_confirmed = True
  planner.starpilot_vcruise.forcing_stop = True
  sm["starpilotPlan"].forcingStop = True
  sm["carState"].standstill = True
  drive(ccm, sm, toggles, clock, 0.0, steps=20)
  assert ccm.experimental_mode

  drive(ccm, sm, toggles, clock, 1.6)
  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["SPEED"]


def test_ccm_launch_assist_holds_chill_until_launch_speed_then_hands_back(monkeypatch):
  planner, detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  stopped_behind_red_light_lead(planner, detector, sm)
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  sm["starpilotPlan"].redLight = False
  detector.stop_light_detected = False
  planner.model_stopped = False

  for mph in range(4, 15):
    drive(ccm, sm, toggles, clock, mph * CV.MPH_TO_MS, steps=4)
    assert not ccm.experimental_mode, mph
    assert ccm.status_value == CCStatus["LEAD"], mph

  # Past 15 mph the normal rules apply again: a slow lead and adjacent car are vetoes.
  drive(ccm, sm, toggles, clock, 16 * CV.MPH_TO_MS)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_launch_assist_exits_on_stop_flag_after_leaving_the_intersection(monkeypatch):
  planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  planner.starpilot_vcruise.stop_sign_confirmed = True
  sm["carState"].standstill = True
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  drive(ccm, sm, toggles, clock, 1.6)
  assert not ccm.experimental_mode

  # 4 m/s for 4 s = 16 m: still inside the stop-flag distance.
  drive(ccm, sm, toggles, clock, 4.0, dt=0.1, steps=40)
  assert not ccm.experimental_mode

  drive(ccm, sm, toggles, clock, 4.0, dt=0.1, steps=12)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_launch_assist_exits_on_new_red_light_past_the_intersection(monkeypatch):
  _planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  drive(ccm, sm, toggles, clock, 5.0, dt=0.1, steps=45)
  assert not ccm.experimental_mode

  sm["starpilotPlan"].redLight = True
  drive(ccm, sm, toggles, clock, 5.0)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def lead_ahead(planner, v_lead, a_lead=0.0):
  planner.tracking_lead = True
  planner.lead_one.status = True
  planner.lead_one.dRel = 12.0
  planner.lead_one.vLead = v_lead
  planner.lead_one.vRel = 0.0
  planner.lead_one.aLeadK = a_lead
  planner.lead_one.radar = True


def test_ccm_launch_assist_hands_back_when_queued_lead_slows_to_a_stop(monkeypatch):
  # STATUS 261 311 replay: Chill stopped ~1.5 m closer behind a queued lead than experimental.
  planner, detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  lead_ahead(planner, 0.0)
  detector.slow_lead_detected = True
  drive(ccm, sm, toggles, clock, 0.0, steps=10)

  lead_ahead(planner, 4.0, 0.5)
  detector.slow_lead_detected = False
  drive(ccm, sm, toggles, clock, 1.6)
  assert not ccm.experimental_mode
  drive(ccm, sm, toggles, clock, 4.0, dt=0.1, steps=60)
  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["LEAD"]

  # Lead slowing but still above the stopping speed: stay in Chill.
  lead_ahead(planner, 2.5, -1.0)
  drive(ccm, sm, toggles, clock, 3.5)
  assert not ccm.experimental_mode

  lead_ahead(planner, 1.8, -1.0)
  drive(ccm, sm, toggles, clock, 3.0)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_launch_assist_keeps_chill_behind_slow_lead_inside_intersection(monkeypatch):
  planner, detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  lead_ahead(planner, 0.0)
  detector.slow_lead_detected = True
  drive(ccm, sm, toggles, clock, 0.0, steps=10)

  lead_ahead(planner, 1.5, -0.5)
  drive(ccm, sm, toggles, clock, 1.6)
  drive(ccm, sm, toggles, clock, 2.0, dt=0.1, steps=20)
  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["LEAD"]


def test_ccm_launch_assist_steady_slow_lead_past_intersection_stays_chill(monkeypatch):
  planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  lead_ahead(planner, 1.8, 0.0)
  drive(ccm, sm, toggles, clock, 1.6)
  drive(ccm, sm, toggles, clock, 2.0, dt=0.1, steps=150)
  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["LEAD"]


def test_ccm_launch_assist_ends_if_car_stops_again_and_rearms(monkeypatch):
  _planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  drive(ccm, sm, toggles, clock, 1.6, steps=4)
  assert not ccm.experimental_mode

  drive(ccm, sm, toggles, clock, 0.0)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]

  drive(ccm, sm, toggles, clock, 0.0, steps=10)
  drive(ccm, sm, toggles, clock, 1.6)
  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["SPEED"]


def test_ccm_launch_assist_keeps_blinker_veto(monkeypatch):
  _planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  sm["carState"].leftBlinker = True
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  drive(ccm, sm, toggles, clock, 1.6)
  assert ccm.experimental_mode


def test_ccm_launch_assist_needs_a_stop_first(monkeypatch):
  _planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch)
  drive(ccm, sm, toggles, clock, 5.0, steps=20)
  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_respects_manual_chill_override(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  planner.params_memory.put_int("CCStatus", CCStatus["USER_CHILL"])

  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: 1.0)
  ccm.update(55 * CV.MPH_TO_MS, 60 * CV.MPH_TO_MS, sm, toggles)

  assert not ccm.experimental_mode
  assert ccm.status_value == CCStatus["USER_CHILL"]


def test_ccm_launch_assist_is_disabled_by_default(monkeypatch):
  _planner, _detector, ccm, sm, toggles, clock = launch_ccm(monkeypatch, t=50.0)
  toggles.conditional_chill_launch_assist = False
  drive(ccm, sm, toggles, clock, 0.0, steps=5)
  drive(ccm, sm, toggles, clock, 1.6, steps=5)

  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]


def test_ccm_restores_persisted_manual_experimental_override(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_sm()
  toggles = make_toggles()
  planner.params.put_bool("PersistChillState", True)
  planner.params.put_int("PersistedCCStatus", CCStatus["USER_EXPERIMENTAL"])

  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: 1.0)
  ccm.update(55 * CV.MPH_TO_MS, 60 * CV.MPH_TO_MS, sm, toggles)

  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["USER_EXPERIMENTAL"]
  assert planner.params_memory.get_int("CCStatus") == CCStatus["USER_EXPERIMENTAL"]


def test_ccm_adjacent_lead_veto_works_with_submaster_like_input(monkeypatch):
  planner, _detector, ccm = make_ccm()
  sm = make_submaster_sm()
  toggles = make_toggles()
  sm["starpilotRadarState"].leadLeft = SimpleNamespace(status=True, dRel=20.0, vLead=12.0)

  monkeypatch.setattr("openpilot.starpilot.controls.lib.conditional_chill_mode.time.monotonic", lambda: 10.0)
  ccm.update(60 * CV.MPH_TO_MS, 65 * CV.MPH_TO_MS, sm, toggles)

  assert ccm.experimental_mode
  assert ccm.status_value == CCStatus["OFF"]
