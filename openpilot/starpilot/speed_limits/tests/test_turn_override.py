"""A turn onto a road Mapbox says is slower replaces the held sign (map_observation.TurnOverride).

Route 00000099--c504cef288: 50 mph sign held, turned into a 25 mph neighborhood,
Mapbox 25 at 23:09, selected stayed 50 until vision read a 20 sign at 23:28.
"""
import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.speed_limits import acceptance as acc, selection as sel
from openpilot.starpilot.speed_limits.map_observation import TurnOverride
from openpilot.starpilot.speed_limits.runtime import Runtime
from openpilot.starpilot.speed_limits.runtime_settings import parse
from openpilot.starpilot.speed_limits.tests.test_navigation_limit import MPH, nav_message
from openpilot.starpilot.speed_limits.vision.observation import MODEL_ID

UNKNOWN = acc.Observation(acc.ObservationKind.UNKNOWN)


def vision(mph, sign='s1'):
  return acc.Observation(acc.ObservationKind.VALID, acc.Candidate('vision',
    acc.ObservationIdentity(acc.IdentityKind.PRODUCER_EPISODE, value=sign), mph * MPH))


def online(mph):
  return acc.Observation(acc.ObservationKind.VALID, acc.Candidate('online',
    acc.ObservationIdentity(acc.IdentityKind.GEOGRAPHIC, value=f'route-{mph}'), mph * MPH))


def drive(rule, steps):
  """steps: (vision, online, heading); returns the last answer."""
  out = None
  for v, o, h in steps:
    out = rule.step(v, o, h)
  return out


def turned_onto_25(rule):
  # 50 sign on a road Mapbox also calls 50, heading east; turn south; road_limit drops, then 25.
  return drive(rule, [(vision(50), online(50), 90.), (vision(50), online(50), 92.), (vision(50), online(50), 180.),
                      (vision(50), UNKNOWN, 180.), (vision(50), online(25), 181.)])


def test_turn_then_lower_mapbox_overrides():
  assert turned_onto_25(TurnOverride())


def test_straight_road_lower_mapbox_keeps_vision():
  rule = TurnOverride()
  # Mapbox under the sign on the same road (45 under a 50), drifting a few degrees, even after a gap.
  assert not drive(rule, [(vision(50), online(45), 90.), (vision(50), UNKNOWN, 100.), (vision(50), online(25), 80.),
                          (vision(50), online(25), 140.)])  # 50 deg is a curve, not another road
  assert not drive(rule, [(vision(50), online(25), None)] * 3)  # no heading (stopped or no fix): no turn


def test_turn_then_higher_mapbox_keeps_vision():
  rule = TurnOverride()
  assert not drive(rule, [(vision(50), online(50), 90.), (vision(50), UNKNOWN, 180.), (vision(50), online(65), 180.)])
  assert not rule.step(vision(50), online(50.5), 180.)  # within SAME_LIMIT_MPS of the sign is not lower


def test_turn_with_pre_turn_mapbox_value_keeps_vision():
  rule = TurnOverride()
  # Mapbox already said 25 before the turn and stays VALID across it: that is the old road's value.
  assert not drive(rule, [(vision(50), online(25), 90.), (vision(50), online(25), 180.), (vision(50), online(25), 270.)])


def test_value_arriving_with_the_turn_counts():
  rule = TurnOverride()
  assert drive(rule, [(vision(50), online(50), 90.), (vision(50), online(25), 180.)])


def test_override_ends_on_a_new_sign():
  rule = TurnOverride()
  assert turned_onto_25(rule)
  assert rule.step(vision(50), online(25), 181.)  # the same 50 republished is not a new sign
  assert not rule.step(vision(20, sign='s2'), online(25), 181.)  # 23:28: camera read the 20 sign
  assert not rule.step(vision(50, sign='s3'), online(25), 181.)  # and the next sign is believed too


def test_override_ends_when_mapbox_goes_unknown_or_stops_being_lower():
  rule = TurnOverride()
  assert turned_onto_25(rule)
  assert not rule.step(vision(50), UNKNOWN, 181.)
  assert not rule.step(vision(50), online(25), 181.)  # coming back needs another turn: no 50/25 flapping
  rule = TurnOverride()
  assert turned_onto_25(rule)
  assert not rule.step(vision(50), online(50), 181.)
  assert not rule.step(vision(50), online(25), 181.)


def test_override_re_arms_after_another_turn():
  rule = TurnOverride()
  assert turned_onto_25(rule)
  assert not rule.step(vision(50), UNKNOWN, 181.)
  assert drive(rule, [(vision(50), UNKNOWN, 270.), (vision(50), online(30), 270.)])


def test_override_holds_through_a_vision_dropout_but_never_hides_a_missing_sign():
  rule = TurnOverride()
  assert turned_onto_25(rule)
  assert not rule.step(UNKNOWN, online(25), 181.)
  assert rule.step(vision(50), online(25), 181.)


# Runtime: SLC observations with real messages.

START = 100_000_000_000
BOOT = 5_000_000_000


class SM:
  def __init__(self):
    self.data, self.valid, self.alive, self.updated, self.logMonoTime = {}, {}, {}, {}, {}

  def __getitem__(self, name):
    return self.data[name]

  def put(self, service, value, stamp):
    self.data[service] = value
    self.logMonoTime[service] = stamp
    self.valid[service] = self.alive[service] = self.updated[service] = True


def gps_message(heading, speed=12.):
  gps = messaging.new_message('gpsLocationExternal').gpsLocationExternal
  gps.hasFix, gps.latitude, gps.longitude, gps.horizontalAccuracy = True, 42.3, -71.1, 5.
  gps.speed, gps.bearingDeg, gps.bearingAccuracyDeg, gps.source = speed, heading, 5., 'ublox'
  return gps


def vision_message(mph, now, episode):
  event = messaging.new_message('slcVisionObservation')
  v = event.slcVisionObservation.init('vision')
  v.producerSessionId, v.status, v.stream, v.modelId = 'camera', 'valid', 'road', MODEL_ID
  v.speedMps, v.confidence, v.supportCount, v.episode, v.frameId = mph * MPH, .9, 2, episode, now // 50_000_000
  v.observedMonoTime, v.validUntilMonoTime, v.cameraFrameEofBootTime = now, now + 250_000_000, BOOT + now - START
  return event.slcVisionObservation


class Drive:
  def __init__(self):
    settings = parse({'SpeedLimitController': True, 'SLCPriority1': 'Dashboard', 'SLCPriority2': 'Vision'})
    assert settings.selection.online_fallback
    self.settings = settings
    self.runtime = Runtime(settings, session_id='drive', vision_enabled=True, vision_control_qualified=True)
    self.sm, self.now, self.cp = SM(), START, type('CP', (), {'carFingerprint': 'civic'})()

  def tick(self, sign_mph, route_mph, heading, *, episode=1, speed=12.):
    self.now += 500_000_000
    self.sm.put('slcVisionObservation', vision_message(sign_mph, self.now, episode), self.now)
    self.sm.put('starpilotNavigation', nav_message(route_mph * MPH, control_valid=route_mph > 0, frame_ns=self.now), self.now)
    self.sm.put('gpsLocationExternal', gps_message(heading, speed), self.now)
    values = self.runtime._observations(self.sm, self.cp, self.now, BOOT + self.now - START)
    return values, sel.select_limit(values, self.settings.selection)


def test_runtime_turn_into_lower_route_limit_selects_online(monkeypatch):
  monkeypatch.setattr('openpilot.starpilot.gps.source.gps_mode', lambda now=None: 'backup')
  car = Drive()
  values, chosen = car.tick(50, 50, 90.)
  assert values[sel.Source.VISION].kind is acc.ObservationKind.VALID and chosen.selected_source is sel.Source.VISION
  car.tick(50, 50, 180.)
  values, chosen = car.tick(50, 0, 180.)  # road_limit dropped its answer at the turn
  assert chosen.selected_source is sel.Source.VISION
  values, chosen = car.tick(50, 25, 180.)
  assert chosen.selected_source is sel.Source.ONLINE and chosen.reason == 'online_fallback'
  assert chosen.observation.candidate.speed_mps == pytest.approx(25 * MPH)
  assert car.runtime.vision_held.kind is acc.ObservationKind.VALID  # the drawer still shows the 50 sign
  # The camera reads the 20 sign: vision again.
  values, chosen = car.tick(20, 25, 180., episode=2)
  assert chosen.selected_source is sel.Source.VISION and chosen.observation.candidate.speed_mps == pytest.approx(20 * MPH)
  assert car.runtime.vision_held.kind is acc.ObservationKind.UNKNOWN


def test_runtime_straight_road_and_slow_turns_keep_vision(monkeypatch):
  monkeypatch.setattr('openpilot.starpilot.gps.source.gps_mode', lambda now=None: 'backup')
  car = Drive()
  for route, heading in ((50, 90.), (0, 95.), (25, 85.), (25, 90.)):
    assert car.tick(50, route, heading)[1].selected_source is sel.Source.VISION
  car = Drive()
  # Turning at walking pace: GPS bearing is noise, no turn is counted.
  for route, heading in ((50, 90.), (0, 180.), (25, 270.)):
    assert car.tick(50, route, heading, speed=2.)[1].selected_source is sel.Source.VISION
  car.runtime.reset()
  assert car.runtime.turn_override.sign is None


def test_runtime_slc_state_reports_online_and_the_held_sign(monkeypatch):
  """End to end through Runtime.step: slcState says online while the sign row still reads 50."""
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages
  from openpilot.starpilot.speed_limits.tests.test_runtime_replay import ReplaySM
  monkeypatch.setattr('openpilot.starpilot.gps.source.gps_mode', lambda now=None: 'backup')
  cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  data, _ = messages()
  data['slcDashboardObservation'] = messaging.new_message('slcDashboardObservation').slcDashboardObservation
  runtime = Runtime(parse({'SpeedLimitController': True, 'SLCPriority1': 'Dashboard', 'SLCPriority2': 'Vision',
                           'SLCConfirmationLower': True}), session_id='turn-drive', vision_enabled=True,
                    vision_control_qualified=True)
  state = None
  for i, (route, heading) in enumerate([(50, 90.)] * 4 + [(50, 180.), (0, 180.)] + [(25, 180.)] * 6):
    now = START + i * 500_000_000
    data['slcVisionObservation'] = vision_message(50, now, 1)
    data['starpilotNavigation'] = nav_message(route * MPH, control_valid=route > 0, frame_ns=now)
    data['gpsLocationExternal'] = gps_message(heading)
    sm = ReplaySM(data, now)
    for service in ('slcVisionObservation', 'starpilotNavigation', 'gpsLocationExternal'):
      sm.logMonoTime[service] = now
      sm.valid[service] = sm.alive[service] = sm.updated[service] = True
    sm['carState'].vCruise = sm['carState'].vCruiseCluster = 90.0
    sm['carState'].vEgoCluster = 12.0
    sm['carControl'].enabled = sm['carControl'].longActive = True
    state = runtime.step(sm, cp, now_ns=now, now_boot_ns=BOOT + now - START).message.slcState
    if i < 6:
      assert str(state.source) != 'online'
  assert str(state.source) == 'online'
  assert abs(state.speedLimit - 25 * MPH) < .001
  rows = {str(row.source): row for row in state.sourceReadings}
  assert str(rows['vision'].observationKind) == 'valid' and abs(rows['vision'].speedLimit - 50 * MPH) < .001
