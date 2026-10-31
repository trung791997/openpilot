from concurrent.futures import Future
import json
from types import SimpleNamespace

import pytest

from openpilot.starpilot.navigation.owner import NavigationOwner
from openpilot.starpilot.navigation.route_engine import MapboxRouteEngine, NavigationRoute, congestion_levels
from openpilot.starpilot.navigation.runtime import REFRESH_NS, RouteRuntime


def route_data(points=5, congestion=None, maxspeed=None):
  annotation = {}
  if congestion is not None:
    annotation['congestion'] = congestion
  if maxspeed is not None:
    annotation['maxspeed'] = maxspeed
  return {'distance': 1112. * (points - 1) / 4, 'duration': 100., 'geometry': {'coordinates': [[.0025 * i, 0.] for i in range(points)]},
          'legs': [{'annotation': annotation, 'steps': [
            {'distance': 1112. * (points - 1) / 4, 'duration': 100., 'maneuver': {'type': 'depart', 'instruction': 'Head east'}},
            {'distance': 0., 'duration': 0., 'maneuver': {'type': 'arrive', 'instruction': 'You have arrived'}}]}]}


class Response:
  def __init__(self, body, status=200):
    self.status_code, self.raw = status, json.dumps(body).encode()
  def __enter__(self):
    return self
  def __exit__(self, *_):
    return False
  def iter_content(self, _size):
    yield self.raw


class Session:
  def __init__(self, *bodies):
    self.bodies, self.calls = list(bodies), []
  def get(self, url, params, timeout, stream):
    self.calls.append((url, params))
    body = self.bodies.pop(0)
    return body if isinstance(body, Response) else Response(body)


class Budget:
  def __init__(self):
    self.spent = []
  def spend(self, enforce=True):
    self.spent.append(enforce)


def test_congestion_parses_per_segment_and_bad_data_is_all_unknown():
  rows = ['low', 'moderate', 'heavy', 'severe']
  assert congestion_levels(route_data(congestion=rows), 4) == [1, 2, 3, 4]
  assert congestion_levels(route_data(congestion=['unknown', 'jam', None, 'low']), 4) == [0, 0, 0, 1]
  assert congestion_levels(route_data(congestion=rows[:3]), 4) == [0] * 4  # mismatched
  assert congestion_levels(route_data(), 4) == [0] * 4                     # missing
  assert congestion_levels({'legs': []}, 4) == [0] * 4
  assert congestion_levels(route_data(congestion='heavy'), 4) == [0] * 4


def test_preview_congestion_takes_the_worst_level_in_each_stride():
  route = NavigationRoute(route_data(points=5, congestion=['low', 'heavy', 'moderate', 'low']))
  assert route.preview_congestion() == [1, 3, 2, 1, 1]  # stride 1: one per segment, the last point repeats
  segments = 1100  # stride 3 over 511 previews
  levels = ['low'] * segments
  levels[4], levels[segments - 1] = 'severe', 'moderate'
  route = NavigationRoute(route_data(points=segments + 1, congestion=levels))
  preview, congestion = route.preview(), route.preview_congestion()
  assert len(preview) == len(congestion)
  assert congestion[0] == 1 and congestion[1] == 4 and congestion[-1] == congestion[-2] == 2


def test_fetch_asks_for_congestion_and_refresh_and_keeps_the_uuid():
  data = route_data(congestion=['low', 'low', 'heavy', 'low'], maxspeed=[{'speed': 50, 'unit': 'km/h'}] * 4)
  session, budget = Session({'code': 'Ok', 'uuid': 'abc-123_X', 'routes': [data, route_data()]}), Budget()
  route = MapboxRouteEngine(session, budget).fetch('pk.test', (0., 0.), {'longitude': .01, 'latitude': 0.})
  params = session.calls[0][1]
  assert set(params['annotations'].split(',')) == {'maxspeed', 'congestion'} and params['enable_refresh'] == 'true'
  assert route.congestion == [1, 1, 3, 1] and route.speed_limits[0] == pytest.approx(50 / 3.6)
  assert [(row.uuid, row.route_index) for row in route.alternatives] == [('abc-123_X', 0), ('abc-123_X', 1)]
  bad = MapboxRouteEngine(Session({'code': 'Ok', 'uuid': '../x', 'routes': [data]})).fetch('pk', (0., 0.), {'longitude': .01, 'latitude': 0.})
  assert bad.uuid is None  # never put into the refresh URL


def refreshed(congestion):
  return {'code': 'Ok', 'route': {'legs': [{'annotation': {'congestion': congestion}}]}}


def test_refresh_merges_from_the_current_index_and_never_touches_guidance():
  route = NavigationRoute(route_data(congestion=['low'] * 4))
  route.uuid, route.route_index = 'abc', 1
  points, steps, limits = list(route.points), list(route.steps), list(route.speed_limits)
  session, budget = Session(refreshed(['heavy', 'severe']), refreshed(['moderate'] * 4)), Budget()
  engine = MapboxRouteEngine(session, budget)
  assert engine.refresh('pk.test', route, 2)
  url, params = session.calls[0]
  assert url == 'https://api.mapbox.com/directions-refresh/v1/mapbox/driving-traffic/abc/1/0'
  assert params['current_route_geometry_index'] == 2 and budget.spent == [False]
  assert route.congestion == [1, 1, 3, 4]
  assert engine.refresh('pk.test', route, 1)  # a whole-route annotation is taken whole
  assert route.congestion == [2, 2, 2, 2]
  assert (route.points, route.steps, route.speed_limits) == (points, steps, limits)


@pytest.mark.parametrize('body', [Response({}, status=401), {'code': 'NotFound'}, {'code': 'Ok'},
                                  refreshed(['heavy'] * 3), refreshed('heavy')])
def test_refresh_errors_keep_the_old_colours(body):
  route = NavigationRoute(route_data(congestion=['low', 'moderate', 'low', 'low']))
  route.uuid = 'abc'
  assert not MapboxRouteEngine(Session(body)).refresh('pk.test', route, 2)
  assert route.congestion == [1, 2, 1, 1]


def test_refresh_needs_a_uuid_and_an_index_on_the_route():
  route, session = NavigationRoute(route_data()), Session()
  assert not MapboxRouteEngine(session).refresh('pk', route, 0) and not session.calls
  route.uuid = 'abc'
  assert not MapboxRouteEngine(session).refresh('pk', route, 4) and not session.calls


class Executor:
  def __init__(self):
    self.jobs = []
  def submit(self, fn, *args):
    future = Future()
    self.jobs.append((fn, args, future))
    return future


def test_runtime_publishes_traffic_and_refreshes_every_five_minutes(tmp_path):
  owner = NavigationOwner(tmp_path, runtime_source=lambda: None)
  owner._is_metric = lambda: True
  settings = owner.configure({'enabled': True, 'token': 'pk.test'}, '0', True)
  owner.select({'name': 'Destination', 'longitude': .01, 'latitude': 0.}, settings['revision'], True)
  route = NavigationRoute(route_data(congestion=['low', 'heavy', 'low', 'low']))
  route.uuid = 'abc'
  calls = []
  def refresh(token, row, segment):
    calls.append((token, segment))
    row.congestion = [4] * 4
    return True
  executor = Executor()
  runtime = RouteRuntime(owner, engine=SimpleNamespace(fetch=None, refresh=refresh), executor=executor)
  start, drive = 10_000_000_000, 9_000_000_000
  def update(t, at=.003):
    return runtime.update(t, (t - 1_000_000, (at, 0.), 5., 90.), drive)
  update(start)
  executor.jobs.pop()[2].set_result(route)
  result = update(start + 1_000_000_000)
  assert result['status'] == 'guiding' and result['routeCongestion'] == [1, 3, 1, 1, 1]
  assert len(result['routeCongestion']) == len(result['route'])
  update(start + 2_000_000_000 + REFRESH_NS // 2)
  assert not executor.jobs  # not yet
  update(start + 2_000_000_000 + REFRESH_NS, at=.006)
  fn, args, future = executor.jobs.pop()
  future.set_result(fn(*args))
  assert calls == [('pk.test', 2)]
  assert update(start + 3_000_000_000 + REFRESH_NS)['routeCongestion'] == [4] * 5
  assert not executor.jobs


def test_runtime_survives_a_refresh_that_raises(tmp_path):
  from openpilot.starpilot.navigation.runtime import _quiet_refresh
  def broken(*_):
    raise OSError('offline')
  assert _quiet_refresh(broken, 'pk', None, 0) is False
