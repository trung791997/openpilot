"""The map overlay is created only when the AA layout enables it and navigation is on, prepared outside the frame,
drawn at its placement, and freed shortly after navigation ends."""
import copy
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from openpilot.starpilot.system.android_auto import projection_onroad as projection
from openpilot.starpilot.system.android_auto.projection_layout import default_layout_for_viewport
from openpilot.starpilot.system.android_auto.tests import test_projection_onroad_boundary as boundary


class FakeMap:
  def __init__(self, *, fonts):
    self.fonts = fonts
    self.prepared, self.drawn, self.closed = [], [], False

  def prepare(self, data, width, height):
    self.prepared.append((data, width, height))

  def draw(self, rect, opacity):
    self.drawn.append((rect, opacity))

  def close(self):
    self.closed = True


IDLE = SimpleNamespace(navigation_active=False, route=())
NAVIGATING = SimpleNamespace(navigation_active=True, route=())
ROUTE_ONLY = SimpleNamespace(navigation_active=False, route=((1.0, 2.0), (1.1, 2.1)))


class FakeFeed:
  def __init__(self):
    self.reads = []
    self.destinations = []
    self.requested = []
    self.input = IDLE

  def read(self, sm, *, navigation_requested=None, destination=None):
    self.reads.append(sm)
    self.destinations.append(destination)
    self.requested.append(navigation_requested)
    return self.input


class FakeLease:
  def __init__(self):
    self.touches, self.releases = 0, 0

  def touch(self):
    self.touches += 1

  def release(self):
    self.releases += 1


class TestProjectionMap(unittest.TestCase):
  def setUp(self):
    self.lease = FakeLease()
    patcher = patch.object(projection, 'MapLeaseWriter', return_value=self.lease)
    patcher.start()
    self.addCleanup(patcher.stop)

  def native(self):
    native, events = boundary.TestProjectionOnroad().dependencies()
    native.map_overlay, native.map_feed = FakeMap, FakeFeed
    return native, events

  def layout(self, **map_placement):
    document = default_layout_for_viewport((2880, 1080))
    document["widgets"]["nav_map"].update(map_placement)
    return document

  def test_disabled_map_builds_nothing(self):
    native, _ = self.native()
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080), customization=self.layout())
    self.assertIsNone(view.map)
    self.assertIsNone(view.map_feed)
    native.ui_state.started = True
    view.prepare()
    self.assertIsNone(view.map)
    self.assertEqual(self.lease.touches, 0)
    view.close()

  def test_placed_map_waits_for_navigation_but_holds_the_lease(self):
    native, _ = self.native()
    layout = self.layout(enabled=True, x=100, y=200, width=640, height=480, opacity=45)
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080), customization=layout)
    self.assertIsInstance(view.map_feed, FakeFeed)
    self.assertIsNone(view.map, "no tile thread or GPU canvases at connect")
    self.assertIsNone(getattr(view.onroad, 'map_layer', None))
    view.prepare()
    self.assertEqual(view.map_feed.reads, [], "nothing to read while offroad")
    native.ui_state.started = True
    for _ in range(3):
      view.prepare()
    self.assertIsNone(view.map, "not navigating: no road map")
    self.assertIsNone(getattr(view.onroad, 'map_layer', None))
    self.assertEqual(len(view.map_feed.reads), 3, "the feed stays warm so the map starts with GPS already known")
    self.assertGreaterEqual(self.lease.touches, 1, "the comma's own screen must not build a second map meanwhile")
    view.close()

  def test_navigation_builds_the_map_and_draws_it_at_placement(self):
    native, _ = self.native()
    layout = self.layout(enabled=True, x=100, y=200, width=640, height=480, opacity=45)
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080), customization=layout)
    native.ui_state.started = True
    view.prepare()
    self.assertIsNone(view.map)
    view.map_feed.input = NAVIGATING
    view.prepare()
    self.assertIsInstance(view.map, FakeMap)
    self.assertIs(view.map.fonts, view.fonts)
    self.assertEqual(view.onroad.map_layer, view._map_layer)
    self.assertEqual(view.map.prepared, [(NAVIGATING, 640, 480)], "prepared on the frame navigation starts")
    self.assertEqual(view.map_feed.reads, [native.ui_state.sm] * 2)
    self.assertEqual(view.map_feed.destinations, [None] * 2, "no favorites: the feed keeps navigation's own name")
    self.assertEqual(view.map_feed.requested, [None] * 2)
    view._map_layer(None, None)
    self.assertEqual(view.map.drawn, [((100, 200, 640, 480), 0.45)])
    overlay = view.map
    view.close()
    self.assertTrue(overlay.closed)

  def test_a_route_without_a_requested_destination_still_shows_the_map(self):
    native, _ = self.native()
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080),
                                       customization=self.layout(enabled=True))
    native.ui_state.started = True
    view.map_feed.input = ROUTE_ONLY
    view.prepare()
    self.assertIsInstance(view.map, FakeMap)
    self.assertEqual(view.map.prepared[-1][0], ROUTE_ONLY)
    view.close()

  def test_navigation_ending_frees_the_map_after_a_hold_and_restarting_rebuilds_it(self):
    native, _ = self.native()
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080),
                                       customization=self.layout(enabled=True))
    native.ui_state.started = True
    clock = [1_000_000_000]
    with patch.object(projection.time, 'monotonic_ns', side_effect=lambda: clock[0]):
      view.map_feed.input = NAVIGATING
      view.prepare()
      overlay = view.map
      view.map_feed.input = IDLE
      view.prepare()
      clock[0] += projection.MAP_RELEASE_AFTER_NAVIGATION_NS // 2
      view.map_feed.input = NAVIGATING
      view.prepare()
      view.map_feed.input = IDLE
      view.prepare()
      clock[0] += projection.MAP_RELEASE_AFTER_NAVIGATION_NS - 1
      view.prepare()
      self.assertIs(view.map, overlay, "a brief gap in navigation keeps the same map")
      self.assertFalse(overlay.closed)
      self.assertEqual(len(overlay.prepared), 5, "drawn unchanged during the hold")
      touches = self.lease.touches
      clock[0] += 1
      view.prepare()
      self.assertTrue(overlay.closed)
      self.assertIsNone(view.map)
      self.assertIsNone(view.onroad.map_layer)
      self.assertIsInstance(view.map_feed, FakeFeed, "the feed stays while the map is placed")
      self.assertEqual(self.lease.releases, 0, "the lease is kept while the layout places the map")
      clock[0] += 2_000_000_000
      view.prepare()
      self.assertGreater(self.lease.touches, touches)
      view.map_feed.input = NAVIGATING
      view.prepare()
      self.assertIsInstance(view.map, FakeMap)
      self.assertIsNot(view.map, overlay)
    view.close()

  def test_going_offroad_frees_the_map_after_the_hold(self):
    native, _ = self.native()
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080),
                                       customization=self.layout(enabled=True))
    native.ui_state.started = True
    clock = [1_000_000_000]
    with patch.object(projection.time, 'monotonic_ns', side_effect=lambda: clock[0]):
      view.map_feed.input = NAVIGATING
      view.prepare()
      overlay = view.map
      native.ui_state.started = False
      view.prepare()
      self.assertIs(view.map, overlay)
      clock[0] += projection.MAP_RELEASE_AFTER_NAVIGATION_NS
      view.prepare()
      self.assertTrue(overlay.closed)
      self.assertIsNone(view.map)
    view.close()

  def test_older_dependencies_without_a_map_still_work(self):
    native, _ = self.native()
    del native.map_overlay
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080),
                                       customization=self.layout(enabled=True))
    self.assertIsNone(view.map)
    self.assertIsNone(view.map_feed)
    native.ui_state.started = True
    view.prepare()
    self.assertIsNone(view.map)
    view.close()

  def test_layout_document_is_not_mutated(self):
    native, _ = self.native()
    layout = self.layout(enabled=True)
    before = copy.deepcopy(layout)
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080), customization=layout)
    native.ui_state.started = True
    view.prepare()
    view.map_feed.input = NAVIGATING
    view.prepare()
    view.close()
    self.assertEqual(layout, before)

  def test_turning_the_map_off_frees_it_and_the_lease_until_it_returns(self):
    native, _ = self.native()
    layout = self.layout(enabled=True)
    view = projection.ProjectionOnroad(dependencies=native, viewport=(2880, 1080), customization=layout)
    native.ui_state.started = True
    view.map_feed.input = NAVIGATING
    view.prepare()
    self.assertEqual(self.lease.touches, 1)
    overlay = view.map
    off = copy.deepcopy(layout)
    off["widgets"]["nav_map"]["enabled"] = False
    view.apply_layout(off)
    view.prepare()
    self.assertTrue(overlay.closed)
    self.assertIsNone(view.map)
    self.assertIsNone(view.map_feed)
    self.assertIsNone(view.onroad.map_layer)
    self.assertGreaterEqual(self.lease.releases, 1)
    view.apply_layout(layout)
    self.assertIsInstance(view.map_feed, FakeFeed)
    self.assertIsNone(view.map, "back on, the road map still waits for navigation")
    view.map_feed.input = NAVIGATING
    view.prepare()
    self.assertIsInstance(view.map, FakeMap)
    self.assertIsNot(view.map, overlay)
    view.close()
