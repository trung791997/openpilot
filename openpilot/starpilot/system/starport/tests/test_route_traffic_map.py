"""Traffic on the car screen's route line: Navigation.routeCongestion -> coloured, batched route strips.

Pure logic (no GL): which colour each stretch of the route gets, that runs of one colour are one strip, and that
missing or mismatched traffic draws the plain route exactly as before.
"""
import time
from types import SimpleNamespace as ns
from unittest.mock import Mock

import numpy as np
import pytest

from openpilot.starpilot.navigation.road_tiles import lat_lon, world_xy
from openpilot.starpilot.ui import onroad_map
from openpilot.starpilot.ui.onroad_map import (
  CONGESTION_CORE, CONGESTION_GLOW, ROUTE_CORE, ROUTE_GLOW, ZOOM_SLOW_M_PER_PX, CanvasRequest, MapOverlay, core_colors, premultiplied,
  route_ahead, route_guidance, segment_levels, split_route,
)

CENTER = (36.1147, -115.1728)
AMBER, ORANGE, RED = (CONGESTION_CORE[level] for level in (2, 3, 4))
GLOW_AMBER, GLOW_ORANGE, GLOW_RED = (CONGESTION_GLOW[level] for level in (2, 3, 4))


def colour(rgba):
  c = premultiplied(rgba)
  return (c.r, c.g, c.b, c.a)


def layer_colours(layers):
  return [(c.r, c.g, c.b, c.a) for c, _ in layers]


def test_head_unit_colours():
  # The head unit's amber and orange; severe is one shade brighter than its #c62828 so it reads on the dark map.
  assert AMBER[:3] == (0xff, 0xb3, 0x00) and ORANGE[:3] == (0xff, 0x57, 0x22) and RED[:3] == (0xe5, 0x39, 0x35)
  assert set(CONGESTION_CORE) == {2, 3, 4}, "unknown (0) and low (1) keep the route colour"


def straight_route(n=200):
  x, y = world_xy(*CENTER)
  world = np.array([(x, y - i * 2e-5) for i in range(n)])
  request = CanvasRequest((x, y), ZOOM_SLOW_M_PER_PX, 1024, None, None, ())
  return world, request


def test_segment_levels_one_per_segment_from_one_per_point():
  route = ((0, 0), (0, 1), (0, 2), (0, 3))
  assert segment_levels(route, (0, 2, 4, 4)).tolist() == [0, 2, 4], "the last point repeats the one before"
  assert segment_levels(route, (1, 3, 9, 3)).tolist() == [1, 3, 0], "an out-of-range level is unknown"


@pytest.mark.parametrize("congestion", [(), (2, 2, 2), (2, 2, 2, 2, 2), (0, 1, 1, 0), (1, 1, 1, 1)])
def test_missing_mismatched_or_light_traffic_is_the_plain_route(congestion):
  assert segment_levels(((0, 0), (0, 1), (0, 2), (0, 3)), congestion) is None
  assert segment_levels(((0, 0),), (2,)) is None


def test_plain_route_strips_are_unchanged_by_the_traffic_path():
  world, request = straight_route()
  plain = route_guidance(world, request, CENTER[0])
  assert layer_colours(plain) == [colour(ROUTE_GLOW), colour(ROUTE_CORE)]
  for levels in (None, np.zeros(len(world) - 1, np.int8), np.ones(len(world) - 1, np.int8), np.full(5, 3, np.int8)):
    same = route_guidance(world, request, CENTER[0], levels)
    assert layer_colours(same) == layer_colours(plain)
    for (_, a), (_, b) in zip(plain, same, strict=True):
      np.testing.assert_array_equal(a, b)


def test_congested_route_is_one_strip_per_colour():
  world, request = straight_route()
  levels = np.zeros(len(world) - 1, np.int8)
  # Many separate stretches of each level along the route: still one strip per colour.
  for start in range(0, len(levels), 20):
    levels[start + 4:start + 8], levels[start + 9:start + 12], levels[start + 14:start + 16] = 2, 3, 4
  layers = route_guidance(world, request, CENTER[0], levels)
  # Every glow first, then every core: a slow stretch's glow takes its colour, and no glow covers a core.
  assert layer_colours(layers) == [colour(c) for c in (ROUTE_GLOW, GLOW_AMBER, GLOW_ORANGE, GLOW_RED,
                                                       ROUTE_CORE, AMBER, ORANGE, RED)]
  # Each glow runs under the same stretches as its core.
  for glow, core in zip(layers[:4], layers[4:], strict=True):
    assert len(glow[1]) == len(core[1])


def test_each_colour_covers_its_own_stretch():
  world, request = straight_route(100)
  levels = np.zeros(99, np.int8)
  levels[10:20], levels[50:60] = 2, 4
  built = route_guidance(world, request, CENTER[0], levels)
  layers = dict(zip(layer_colours(built), (strip for _, strip in built), strict=True))
  points = ((world - request.center) * onroad_map.meters_per_tile(CENTER[0]) * onroad_map.SUPERSAMPLE /
            request.m_per_px + request.side / 2)
  amber, red = layers[colour(AMBER)][:, 1], layers[colour(RED)][:, 1]
  # The route runs north (up the canvas): y falls with the point index.
  assert points[20, 1] - 1 < amber.min() and amber.max() < points[10, 1] + 1
  assert points[60, 1] - 1 < red.min() and red.max() < points[50, 1] + 1
  assert colour(ORANGE) not in layers, "no heavy traffic, no orange strip"


def test_core_colours_without_matching_levels_fall_back_to_one_plain_mask():
  keep = np.array([True, True, False, True])
  (only, mask), = core_colors(keep, None)
  assert only == ROUTE_CORE and mask is keep
  (only, mask), = core_colors(keep, np.array([2, 2], np.int8))
  assert only == ROUTE_CORE and mask is keep
  result = core_colors(keep, np.array([0, 2, 2, 4], np.int8))
  assert [c for c, _ in result] == [ROUTE_CORE, AMBER, RED]
  assert [m.tolist() for _, m in result] == [[True, False, False, False], [False, True, False, False],
                                             [False, False, False, True]], "off-canvas segments stay culled"


def test_route_ahead_keeps_the_levels_of_the_segments_ahead():
  route = np.array([[0, 0], [0, 10], [0, 20], [0, 30]], float)
  levels = np.array([2, 3, 4], np.int8)
  ahead, ahead_levels = route_ahead(route, (0.5, 14.0), levels)
  np.testing.assert_array_equal(ahead, split_route(route, (0.5, 14.0))[1])
  assert ahead_levels.tolist() == [3, 4] and len(ahead_levels) == len(ahead) - 1
  assert route_ahead(route, (0, 0), levels[:2])[1] is None, "mismatched levels: the plain route"
  assert route_ahead(None, (0, 0), levels) == (None, None)


def overlay_on_route(tmp_path, monkeypatch, n=400):
  overlay = MapOverlay(tmp_path, clock=lambda: 100.0)
  x, y = world_xy(*CENTER)
  route = tuple(lat_lon(x, y + 0.02 - i * 1e-4) for i in range(n))
  overlay._latitude, overlay._m_per_px, overlay._world = CENTER[0], ZOOM_SLOW_M_PER_PX, (x, y)
  overlay._canvas_request = CanvasRequest((x, y), ZOOM_SLOW_M_PER_PX, 1024, None, None, ())
  builds = []
  real = onroad_map.route_guidance
  monkeypatch.setattr(onroad_map, "route_guidance", lambda *args: builds.append(args) or real(*args))
  draws = []
  for name in ("begin_texture_mode", "end_texture_mode", "clear_background", "begin_blend_mode", "end_blend_mode",
               "draw_rectangle", "draw_texture_pro", "rl_push_matrix", "rl_pop_matrix", "rl_translatef", "rl_rotatef",
               "rl_scalef", "draw_circle_v", "draw_triangle"):
    monkeypatch.setattr(onroad_map.rl, name, lambda *args: None)
  monkeypatch.setattr(onroad_map.rl, "draw_triangle_strip", lambda points, count, color: draws.append((color.r, color.g, color.b)))
  overlay._canvas = object()
  return overlay, route, builds, draws


def test_overlay_colours_the_route_ahead_and_rebuilds_only_on_new_traffic(tmp_path, monkeypatch):
  overlay, route, builds, draws = overlay_on_route(tmp_path, monkeypatch)
  try:
    congestion = tuple([1] * 220 + [3] * 60 + [1] * 120)        # heavy traffic just ahead of the car (point 200)
    overlay._route(route, congestion)
    overlay._compose_layer(400, 300, True)
    assert len(builds) == 1
    assert layer_colours(overlay._route_layers) == [colour(c) for c in (ROUTE_GLOW, GLOW_ORANGE, ROUTE_CORE, ORANGE)]
    assert len(draws) == 4, "one glow and one core draw per colour per frame, not one per segment"
    overlay._route(route, tuple(congestion))                       # the same levels in a new message
    overlay._compose_layer(400, 300, True)
    assert len(builds) == 1, "unchanged traffic reuses the strips"
    overlay._route(route, tuple([1] * 220 + [4] * 60 + [1] * 120))  # it got worse
    overlay._compose_layer(400, 300, True)
    assert len(builds) == 2 and colour(RED) in layer_colours(overlay._route_layers)
    overlay._route(route, (4,) * 10)                                # mismatched length
    overlay._compose_layer(400, 300, True)
    assert len(builds) == 3 and layer_colours(overlay._route_layers) == [colour(ROUTE_GLOW), colour(ROUTE_CORE)]
    overlay._route(route, ())                                       # gone: still plain, nothing to rebuild
    overlay._compose_layer(400, 300, True)
    assert len(builds) == 3
  finally:
    overlay._canvas = None
    overlay.close()


def test_traffic_behind_the_car_is_not_drawn(tmp_path, monkeypatch):
  overlay, route, _, _ = overlay_on_route(tmp_path, monkeypatch)
  try:
    overlay._route(route, tuple([4] * 150 + [0] * 250))             # the car is at point 200
    overlay._compose_layer(400, 300, True)
    assert colour(RED) not in layer_colours(overlay._route_layers)
  finally:
    overlay._canvas = None
    overlay.close()


class FakeNavigationMaster:
  def __init__(self):
    from openpilot.starpilot.gps.source import GPS_SOURCES
    self.messages, self.valid, self.logMonoTime = {}, {}, {}
    for service in GPS_SOURCES:
      self.valid[service], self.logMonoTime[service] = False, 0

  def update(self, timeout):
    pass

  def __getitem__(self, key):
    return self.messages[key]

  def put(self, stamp, points, congestion=None):
    from openpilot.starpilot.navigation.wire import VERSION
    nav = ns(version=VERSION, status='guiding', route=[ns(latitude=lat, longitude=lon) for lat, lon in points],
             enabled=True, destinationName='Home', frameMonoTime=time.monotonic_ns())
    if congestion is not None:
      nav.routeCongestion = congestion
    self.messages['starpilotNavigation'] = ns(navigation=nav)
    self.valid['starpilotNavigation'], self.logMonoTime['starpilotNavigation'] = True, stamp


def test_map_feed_carries_route_congestion(monkeypatch):
  from openpilot.starpilot.ui.onroad_map import MapFeed
  clock = [100.0]
  monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(time, 'monotonic_ns', lambda: int(clock[0] * 1e9))
  sm = FakeNavigationMaster()
  acquisition = Mock(since=clock[0])
  acquisition.satellites.return_value = None
  feed = MapFeed(sm, acquisition=acquisition)
  points = [(36.0 + i * 1e-4, -115.0) for i in range(4)]
  sm.put(1, points, [0, 2, 3, 3])
  first = feed.read(sm)
  assert first.congestion == (0, 2, 3, 3) and len(first.route) == 4
  sm.put(2, points, [0, 2, 3, 3])
  assert feed.read(sm).congestion is first.congestion, "the same traffic keeps the same tuple (no guidance rebuild)"
  sm.put(3, points, [0, 4, 4, 4])
  assert feed.read(sm).congestion == (0, 4, 4, 4)
  sm.put(4, points)                                                  # an older navigationd: no routeCongestion
  assert feed.read(sm).congestion == () and len(feed.read(sm).route) == 4
  sm.put(5, points, [2, 2, 2, 2])
  clock[0] += 3.1
  assert feed.read(sm).congestion == () and feed.read(sm).route == (), "traffic expires with the route"
