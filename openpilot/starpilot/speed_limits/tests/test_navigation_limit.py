"""Mapbox route speed limit: navigation publishes it, SLC uses it only as the online fallback."""
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.navigation.route_engine import MapboxRouteEngine, NavigationRoute, max_speeds
from openpilot.starpilot.navigation.wire import navigation_envelope
from openpilot.starpilot.speed_limits import acceptance as acc, selection as sel
from openpilot.starpilot.speed_limits.navigation_observation import navigation_observation
from openpilot.starpilot.speed_limits.runtime import Runtime
from openpilot.starpilot.speed_limits.runtime_settings import parse

MONO = 100_000_000_000
MPH = 0.44704


def route_data(maxspeed):
  return {'distance': 2224., 'duration': 100., 'geometry': {'coordinates': [[0., 0.], [.01, 0.], [.02, 0.]]},
          'legs': [{'annotation': {'maxspeed': maxspeed}, 'steps': [
            {'distance': 2224., 'duration': 100., 'maneuver': {'type': 'depart', 'instruction': 'Head east'}},
            {'distance': 0., 'duration': 0., 'maneuver': {'type': 'arrive', 'instruction': 'You have arrived'}}]}]}


def test_mapbox_maxspeed_annotation_is_read_per_segment():
  assert max_speeds(route_data([{'speed': 45, 'unit': 'mph'}, {'speed': 100, 'unit': 'km/h'}]), 2) == \
    pytest.approx([45 * MPH, 100 / 3.6])
  # No posted limit, unknown, nonsense and unit-less entries are all unknown, never "no limit".
  assert max_speeds(route_data([{'none': True}, {'unknown': True}]), 2) == [None, None]
  assert max_speeds(route_data([{'speed': 'fast', 'unit': 'mph'}, {'speed': 45}]), 2) == [None, None]
  assert max_speeds(route_data([{'speed': 0, 'unit': 'mph'}, {'speed': 400, 'unit': 'km/h'}]), 2) == [None, None]
  assert max_speeds(route_data([{'speed': 45, 'unit': 'mph'}]), 2) == [None, None]  # misaligned with the geometry
  assert max_speeds({'legs': [{}]}, 2) == [None, None]


def test_route_progress_reports_the_matched_segment_limit_only_on_route():
  route = NavigationRoute(route_data([{'speed': 45, 'unit': 'mph'}, {'speed': 30, 'unit': 'mph'}]))
  assert route.progress((.005, 0.), 15., 90.).speed_limit_mps == pytest.approx(45 * MPH)
  assert route.progress((.015, 0.), 15., 90.).speed_limit_mps == pytest.approx(30 * MPH)
  assert route.progress((.015, 0.), 15., 270.).speed_limit_mps is None  # driving the other way: off route
  assert route.progress((.015, .01), 15., 90.).speed_limit_mps is None  # a kilometre away: off route
  assert NavigationRoute(route_data(None)).progress((.005, 0.), 15., 90.).speed_limit_mps is None


def test_directions_request_asks_for_maxspeed():
  calls = []
  class Response:
    status_code = 200
    headers = {'content-type': 'application/json'}
    def json(self):
      return {'code': 'Ok', 'routes': [route_data([{'speed': 45, 'unit': 'mph'}, {'unknown': True}])]}
  class Session:
    def get(self, url, params=None, **kwargs):
      calls.append(params)
      return Response()
  try:
    MapboxRouteEngine(Session()).fetch('pk.test', (0., 0.), {'longitude': .02, 'latitude': 0.})
  except Exception:
    pass  # response_json's transport details are covered elsewhere; only the request matters here
  assert calls and calls[0]['annotations'] == 'maxspeed' and calls[0]['overview'] == 'full'


def nav_message(speed_limit, *, status='guiding', control_valid=True, frame_ns=MONO):
  message = messaging.new_message('starpilotNavigation', valid=True)
  message.starpilotNavigation = navigation_envelope({
    'sessionId': 'nav', 'frameMonoTime': frame_ns, 'startedMonoTime': 1, 'revision': 'r1', 'enabled': True,
    'status': status, 'destinationName': 'Home', 'instruction': {}, 'nextManeuver': {}, 'route': [],
    'locationMonoTime': frame_ns, 'controlValid': control_valid, 'speedLimit': speed_limit})
  return message.starpilotNavigation


class NavSM:
  def __init__(self, navigation, stamp=MONO):
    self.data = {'starpilotNavigation': navigation}
    self.valid = {'starpilotNavigation': True}
    self.alive = {'starpilotNavigation': True}
    self.updated = {'starpilotNavigation': True}
    self.logMonoTime = {'starpilotNavigation': stamp}

  def __getitem__(self, name):
    return self.data[name]


def observe(navigation, now=MONO, stamp=MONO):
  return navigation_observation(NavSM(navigation, stamp), now_ns=now, drive_session='drive', car_fingerprint='civic')


def test_navigation_observation_is_valid_only_while_guiding_on_route():
  valid = observe(nav_message(45 * MPH))
  assert valid.kind is acc.ObservationKind.VALID and valid.candidate.source == 'online'
  assert valid.candidate.speed_mps == pytest.approx(45 * MPH)
  assert acc.observation_is_valid(valid)
  assert observe(nav_message(0.)).kind is acc.ObservationKind.UNKNOWN  # Mapbox had no limit here
  assert observe(nav_message(45 * MPH, control_valid=False)).kind is acc.ObservationKind.UNKNOWN  # off route
  assert observe(nav_message(45 * MPH, status='routing')).kind is acc.ObservationKind.ABSENT
  assert observe(nav_message(45 * MPH), now=MONO + 4_000_000_000).kind is acc.ObservationKind.STALE
  assert observe(nav_message(45 * MPH, frame_ns=MONO - 4_000_000_000)).kind is acc.ObservationKind.STALE
  assert navigation_observation(NS(logMonoTime={}, valid={}), now_ns=MONO, drive_session='d',
                                car_fingerprint='c').kind is acc.ObservationKind.ABSENT


def test_online_is_used_only_when_both_slots_have_no_limit():
  settings = parse({'SpeedLimitController': True, 'SLCPriority1': 'Dashboard', 'SLCPriority2': 'Vision'})
  assert settings.selection.online_fallback
  unknown = acc.Observation(acc.ObservationKind.UNKNOWN)
  online = observe(nav_message(45 * MPH))
  vision = acc.Observation(acc.ObservationKind.VALID, acc.Candidate('vision',
    acc.ObservationIdentity(acc.IdentityKind.GEOGRAPHIC, value='sign'), 35 * MPH))
  values = {sel.Source.DASHBOARD: unknown, sel.Source.VISION: unknown, sel.Source.MAP: unknown, sel.Source.ONLINE: online}
  chosen = sel.select_limit(values, settings.selection)
  assert chosen.selected_source is sel.Source.ONLINE and chosen.reason == 'online_fallback'
  chosen = sel.select_limit({**values, sel.Source.VISION: vision}, settings.selection)
  assert chosen.selected_source is sel.Source.VISION


def test_a_seen_sign_holds_until_the_route_limit_changes():
  settings = parse({'SpeedLimitController': True, 'SLCPriority1': 'Dashboard', 'SLCPriority2': 'Vision'})
  runtime = Runtime(settings, session_id='drive')
  cp = NS(carFingerprint='civic')
  sm = NavSM(nav_message(45 * MPH))
  assert runtime._observations(sm, cp, MONO, MONO)[sel.Source.ONLINE].kind is acc.ObservationKind.VALID
  runtime.sign_first_online.sign_seen, runtime.sign_first_online.baseline_mps = True, 45 * MPH
  assert runtime._observations(sm, cp, MONO, MONO)[sel.Source.ONLINE].kind is acc.ObservationKind.UNKNOWN
  changed = runtime._observations(NavSM(nav_message(30 * MPH)), cp, MONO, MONO)[sel.Source.ONLINE]
  assert changed.kind is acc.ObservationKind.VALID and changed.candidate.speed_mps == pytest.approx(30 * MPH)
  # Vision then Maps: the route limit is held behind both.
  runtime = Runtime(parse({'SpeedLimitController': True, 'SLCPriority1': 'Vision', 'SLCPriority2': 'Maps'}),
                    session_id='drive')
  runtime.sign_first_online.sign_seen, runtime.sign_first_online.baseline_mps = True, 45 * MPH
  assert runtime._observations(sm, cp, MONO, MONO)[sel.Source.ONLINE].kind is acc.ObservationKind.UNKNOWN


def test_route_limit_fills_in_and_card_applies_its_command():
  """Peter's slots (Dashboard, Vision), no sign and no dash limit: the route's 45 mph is adopted."""
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages
  from openpilot.starpilot.speed_limits import physical_actions
  from openpilot.starpilot.speed_limits.tests.test_runtime_replay import ReplaySM
  cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  data, _ = messages()
  data['slcDashboardObservation'] = messaging.new_message('slcDashboardObservation').slcDashboardObservation
  limit = 45 * MPH
  runtime = Runtime(parse({'SpeedLimitController': True, 'SLCPriority1': 'Dashboard', 'SLCPriority2': 'Vision',
                           'SLCConfirmationLower': True}), session_id='nav-drive')
  command = None
  for i in range(80):  # 4 s at 20 Hz; navigation itself refreshes at 1 Hz
    dt = i * 50_000_000
    published = MONO + dt - dt % 1_000_000_000
    data['starpilotNavigation'] = nav_message(limit, frame_ns=published)
    sm = ReplaySM(data, MONO + dt)
    sm.logMonoTime['starpilotNavigation'] = published
    sm['carState'].vCruise = sm['carState'].vCruiseCluster = 40.0
    sm['carState'].vEgoCluster = 15.0
    sm['carControl'].enabled = sm['carControl'].longActive = True
    out = runtime.step(sm, cp, now_ns=MONO + dt, now_boot_ns=MONO + dt)
    if out.command is not None:
      command, state, now = out.command.slcCruiseCommand, out.message.slcState, MONO + dt
      break
  assert command is not None and command.source == 'online'
  assert dt >= 3_000_000_000  # the automatic adoption hold still applies
  assert abs(state.acceptedSpeedLimit - limit) < .001
  current = physical_actions.state_current(ReplaySM({'slcState': state}, now), None, 'card-producer', now)
  assert physical_actions.command_applicable(command, current, None, 'card-producer', now, 40.0 / 3.6,
                                             long_active=True, button_event=False)
