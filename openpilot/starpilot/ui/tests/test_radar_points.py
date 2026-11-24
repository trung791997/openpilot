"""Radar Point Display: projection, the per-renderer dots, the saved choice and its settings row."""

import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.starpilot.ui.appearance_owner import AppearanceOwner
from openpilot.starpilot.ui.feature_settings_state import row_change
from openpilot.starpilot.ui.onroad_customization import PARAM_KEY, decode_document, default_document, validate_document
from openpilot.starpilot.ui.presentation import Profile
from openpilot.starpilot.ui.radar_points import (
  FILL_COLOR, FUSED_FILL_COLOR, FUSE_GRACE_NS, SERVICE, XCP_FILL_COLOR, ConfirmTracker, RadarPointsRenderer, YawHistory,
  project_radar_points, radar_points_enabled, rotate_for_yaw,
)


def _scalar_reference(d_rel, in_y, path_x, path_z, transform, path_offset_z, clip_bounds, rect_bounds):
  clip_x, clip_y, clip_xmax, clip_ymax = clip_bounds
  rect_x, rect_y, rect_xmax, rect_ymax = rect_bounds
  result = []
  for d, y in zip(d_rel, in_y, strict=True):
    d, y = float(d), float(y)
    if not math.isfinite(d) or not math.isfinite(y):
      continue
    idx = max(int(np.searchsorted(path_x, d, side="right") - 1), 0)
    in_z = (float(path_z[idx]) if idx < len(path_z) else 0.0) + float(path_offset_z)
    row = [float(transform[i, 0]) * d + float(transform[i, 1]) * y + float(transform[i, 2]) * in_z for i in range(3)]
    if abs(row[2]) < 1e-6:
      continue
    x, screen_y = row[0] / row[2], row[1] / row[2]
    if not (clip_x <= x <= clip_xmax and clip_y <= screen_y <= clip_ymax):
      continue
    result.append((max(rect_x, min(x, rect_xmax)), max(rect_y, min(screen_y, rect_ymax))))
  return np.asarray(result, dtype=np.float64).reshape(-1, 2)


def _inputs(count=65):
  d_rel = np.asarray([4.0 + i * 2.4 for i in range(count)], dtype=np.float64)
  in_y = np.asarray([((i % 9) - 4) * 0.45 for i in range(count)], dtype=np.float64)
  path_x = np.linspace(0.0, 192.0, 33, dtype=np.float32)
  path_z = (0.15 * np.sin(path_x / 30.0)).astype(np.float32)
  transform = np.asarray([[18.0, 0.25, 960.0], [0.1, -16.0, 820.0], [0.045, 0.001, 1.0]], dtype=np.float32)
  return d_rel, in_y, path_x, path_z, transform, (-500.0, -500.0, 2420.0, 1580.0), (0.0, 0.0, 1920.0, 1080.0)


@pytest.mark.parametrize("count", [0, 1, 10, 20, 65])
def test_projection_matches_scalar_reference(count):
  d_rel, in_y, path_x, path_z, transform, clip, rect = _inputs(count)
  expected = _scalar_reference(d_rel, in_y, path_x, path_z, transform, 1.22, clip, rect)
  actual = project_radar_points(d_rel, in_y, path_x, path_z, transform, 1.22, clip, rect)
  assert actual.shape == expected.shape
  np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-10)


def test_projection_keeps_order_and_drops_non_finite_points():
  _, _, path_x, path_z, transform, clip, rect = _inputs(0)
  d_rel = np.asarray([20.0, np.nan, 8.0, 20.0, np.inf, 12.0], dtype=np.float64)
  in_y = np.asarray([1.0, 2.0, -1.0, 1.0, 3.0, -2.0], dtype=np.float64)
  expected = _scalar_reference(d_rel, in_y, path_x, path_z, transform, 1.22, clip, rect)
  actual = project_radar_points(d_rel, in_y, path_x, path_z, transform, 1.22, clip, rect)
  np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-10)
  assert actual.shape[0] == 4
  np.testing.assert_allclose(actual[0], actual[2], rtol=0.0, atol=1e-10)


def test_projection_drops_outside_clip_and_clamps_into_rect():
  d_rel, in_y, path_x, path_z, transform, _, _ = _inputs(30)
  clip, rect = (0.0, 0.0, 560.0, 1000.0), (100.0, 200.0, 540.0, 800.0)
  expected = _scalar_reference(d_rel, in_y, path_x, path_z, transform, 1.22, clip, rect)
  actual = project_radar_points(d_rel, in_y, path_x, path_z, transform, 1.22, clip, rect)
  np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-10)
  assert 0 < actual.shape[0] < 30
  assert np.all((actual[:, 0] >= 100.0) & (actual[:, 0] <= 540.0) & (actual[:, 1] >= 200.0) & (actual[:, 1] <= 800.0))


def test_projection_handles_path_edges_and_the_focal_plane():
  path_x, transform = np.asarray([0.0, 10.0], dtype=np.float32), np.eye(3, dtype=np.float32)
  bounds = (-100.0, -100.0, 100.0, 100.0)
  d_rel, in_y = np.asarray([-5.0, 0.0, 5.0, 25.0]), np.zeros(4)
  path_z = np.asarray([1.0, 2.0], dtype=np.float32)
  np.testing.assert_allclose(project_radar_points(d_rel, in_y, path_x, path_z, transform, 1.0, bounds, bounds),
                             _scalar_reference(d_rel, in_y, path_x, path_z, transform, 1.0, bounds, bounds), atol=1e-10)
  # w == 0: nothing to divide by, so nothing is drawn
  assert project_radar_points(np.asarray([1.0, 2.0]), np.zeros(2), path_x, np.zeros(2, dtype=np.float32), transform, 0.0,
                              bounds, bounds).shape == (0, 2)


class FakeSm(dict):
  def __init__(self, points):
    super().__init__({SERVICE: NS(points=points)})
    self.seen, self.alive, self.valid = {SERVICE: True}, {SERVICE: True}, {SERVICE: True}
    self.recv_frame = {SERVICE: 1, "modelV2": 1}
    self.reads = 0

  def __getitem__(self, key):
    self.reads += 1
    return super().__getitem__(key)


def _renderer_inputs():
  # identity-like transform: x = dRel / z, y = -yRel / z with z = path height + offset = 1
  path = np.asarray([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]], dtype=np.float32)
  view = NS(_path=NS(raw_points=path), _car_space_transform=np.eye(3, dtype=np.float32), _path_offset_z=1.0)
  points = [NS(dRel=10.0, yRel=-20.0), NS(dRel=30.0, yRel=-40.0), NS(dRel=500.0, yRel=-40.0)]
  return view, FakeSm(points), NS(x=5.0, y=7.0, width=100.0, height=100.0)


def test_renderer_is_off_by_default_and_never_reads_messages_while_off():
  view, sm, rect = _renderer_inputs()
  dots = RadarPointsRenderer(7.0, 9.0)
  assert not dots.enabled
  assert dots.screen_points(sm, view, rect).shape == (0, 2)
  dots.draw(sm, view, rect)
  assert sm.reads == 0


def test_renderer_positions_cache_and_staleness():
  view, sm, rect = _renderer_inputs()
  dots = RadarPointsRenderer(7.0, 9.0)
  dots.enabled = True
  # screen coordinates: the third point is off the view and, with no margin, dropped
  np.testing.assert_allclose(dots.screen_points(sm, view, rect), [[10.0, 20.0], [30.0, 40.0]])
  dots.screen_points(sm, view, rect)
  assert sm.reads == 1  # same radar and model frame: reuse the projection
  sm.recv_frame[SERVICE] = 2
  sm[SERVICE].points.pop(0)
  reads = sm.reads
  np.testing.assert_allclose(dots.screen_points(sm, view, rect), [[30.0, 40.0]])
  assert sm.reads == reads + 1
  for flags in (sm.seen, sm.alive, sm.valid):
    flags[SERVICE] = False
    assert dots.screen_points(sm, view, rect).shape == (0, 2)
    flags[SERVICE] = True
  assert dots.screen_points(sm, view, rect).shape == (1, 2)
  view._path.raw_points = np.empty((0, 3), dtype=np.float32)
  assert dots.screen_points(sm, view, rect).shape == (0, 2)


def test_renderer_margin_pins_to_the_edge_and_compact_view_adds_the_rect_origin():
  view, sm, rect = _renderer_inputs()
  pinned = RadarPointsRenderer(7.0, 9.0, clip_margin=500.0)
  pinned.enabled = True
  np.testing.assert_allclose(pinned.screen_points(sm, view, rect), [[10.0, 20.0], [30.0, 40.0], [105.0, 40.0]])
  compact = RadarPointsRenderer(4.0, 5.5, rect_relative=True)
  compact.enabled = True
  np.testing.assert_allclose(compact.screen_points(sm, view, rect), [[15.0, 27.0], [35.0, 47.0]])


def test_saved_choice_is_an_optional_boolean_in_the_layout_document():
  document = default_document()
  assert "radarPoints" not in document and not radar_points_enabled(document)
  assert "radarPoints" not in validate_document(document)
  for value in (True, False):
    assert validate_document({**document, "radarPoints": value})["radarPoints"] is value
    assert radar_points_enabled({**document, "radarPoints": value}) is value
  for value in (1, "true", None):
    with pytest.raises(ValueError):
      validate_document({**document, "radarPoints": value})
    assert not radar_points_enabled({**document, "radarPoints": value})
  assert not radar_points_enabled(None)


@pytest.mark.parametrize("profile", list(Profile))
def test_settings_row_saves_only_while_parked_and_keeps_the_layout(tmp_path, profile):
  class Params:
    def get_param_path(self, key):
      return str(tmp_path / key)
  document = default_document()
  document["layouts"]["large"]["clock"].update(x=700, y=700, enabled=True)
  path = tmp_path / PARAM_KEY
  path.write_text(json.dumps(document))
  parked = True
  owner = AppearanceOwner(Params(), lambda: parked)

  def row():
    return next(row for row in owner.snapshot(profile).rows if row.key == "RadarPoints")

  assert (row().label, row().value, row().default_value, row().available) == ("Radar Point Display", "Off", "Off", True)
  request = row_change(row(), 1)
  assert request is not None and owner.apply(request)
  saved = decode_document(path.read_bytes())
  assert saved["radarPoints"] is True and saved["layouts"] == document["layouts"] and saved["reduceMotion"] is False
  assert row().value == "On"
  assert not owner.apply(request)  # the source it was made from is gone
  request = row_change(row(), -1)
  assert request is not None
  parked = False
  assert not owner.apply(request) and not row().available
  parked = True
  assert owner.apply(request) and decode_document(path.read_bytes())["radarPoints"] is False
  path.write_bytes(b"corrupt layout")
  assert not owner.apply(request) and path.read_bytes() == b"corrupt layout"
  assert (row().value, row().available) == ("Unavailable", False)


def test_xcp_overlay_points_project_cache_and_expire(tmp_path):
  xcp_path = tmp_path / "xcp_radar_points.json"
  xcp_path.write_text(json.dumps({"mono_ns": 10_000_000_000, "points": [[15.0, -25.0], [500.0, -40.0]]}))
  view, sm, rect = _renderer_inputs()
  dots = RadarPointsRenderer(7.0, 9.0, rect_relative=True, xcp_path=xcp_path)
  assert dots.xcp_screen_points(sm, view, rect, now_ns=10_100_000_000).shape == (0, 2)
  dots.enabled = True
  np.testing.assert_allclose(dots.xcp_screen_points(sm, view, rect, now_ns=10_100_000_000), [[20.0, 32.0]])
  # Stale (> 1.0s) or missing snapshot clears points cleanly
  assert dots.xcp_screen_points(sm, view, rect, now_ns=11_200_000_000).shape == (0, 2)
  xcp_path.unlink()
  assert dots.xcp_screen_points(sm, view, rect, now_ns=10_100_000_000).shape == (0, 2)


def _a(*rows):
  return np.asarray(rows, dtype=np.float64).reshape(-1, 2)


def test_draw_splits_fused_cam_only_and_xcp_only(tmp_path, monkeypatch):
  import sys
  calls = []

  class Rl(NS):
    pass

  rl = Rl(Color=lambda *c: tuple(c), Vector2=lambda x, y: (x, y), draw_circle_v=lambda c, r, col: calls.append((c, r, col)))
  monkeypatch.setitem(sys.modules, "openpilot.starpilot.ui.raylib", NS(rl=rl))
  xcp_path = tmp_path / "xcp.json"
  # camera: (10,-20) fused with xcp (11,-20); (30,-40) camera only; xcp (50,-50) radar only
  xcp_path.write_text(json.dumps({"mono_ns": 10_000_000_000, "points": [[11.0, -20.0], [50.0, -50.0]]}))
  view, sm, rect = _renderer_inputs()
  sm[SERVICE].points[:] = [NS(dRel=10.0, yRel=-20.0), NS(dRel=30.0, yRel=-40.0)]
  dots = RadarPointsRenderer(7.0, 9.0, xcp_path=xcp_path)
  dots.enabled = True
  dots.draw(sm, view, rect, now_ns=10_100_000_000)
  fills = [c for c in calls if c[2] in (FILL_COLOR, XCP_FILL_COLOR, FUSED_FILL_COLOR)]
  assert [f[2] for f in fills].count(FUSED_FILL_COLOR) == 1
  assert [f[2] for f in fills].count(FILL_COLOR) == 1
  assert [f[2] for f in fills].count(XCP_FILL_COLOR) == 1
  fused = next(f for f in fills if f[2] == FUSED_FILL_COLOR)
  assert fused[0] == (10.0, 20.0) and fused[1] == 7.0  # at the camera position, full radius
  blue = next(f for f in fills if f[2] == XCP_FILL_COLOR)
  assert blue[1] == max(2.5, 7.0 * 0.72)
  assert len(calls) == 6  # three dots, outline + fill each
  # stale radar: nothing fuses, camera draws red for both points
  calls.clear()
  dots.draw(sm, view, rect, now_ns=12_000_000_000)
  assert [c[2] for c in calls if c[2] in (FILL_COLOR, XCP_FILL_COLOR, FUSED_FILL_COLOR)] == [FILL_COLOR, FILL_COLOR]


def test_xcp_snapshot_reads_new_five_field_points_and_old_two_field_points(tmp_path):
  path = tmp_path / "xcp.json"
  dots = RadarPointsRenderer(7.0, 9.0, xcp_path=path)
  path.write_text(json.dumps({"mono_ns": 10_000_000_000, "points": [[20.0, 1.0, 0.1, 0.4, 31], [30.0, -2.0]]}))
  _, pts = dots._read_xcp_snapshot(10_100_000_000)
  assert pts == [(20.0, 1.0, 0.1, 0.4, 31), (30.0, -2.0, 0.0, 0.0, -2)]  # no id: -1 - index, no ages


def test_yaw_history_integrates_piecewise_and_holds_the_ends():
  yaw = YawHistory()
  assert yaw.angle(0, 1_000_000_000) == 0.0  # no history: no rotation
  yaw.add(1_000_000_000, 0.2)
  yaw.add(1_500_000_000, -0.1)
  # 0.2 rad/s for 0.5 s, then -0.1 rad/s for 0.25 s
  assert yaw.angle(1_000_000_000, 1_750_000_000) == pytest.approx(0.2 * 0.5 - 0.1 * 0.25)
  assert yaw.angle(500_000_000, 1_000_000_000) == pytest.approx(0.2 * 0.5)  # first sample held back
  assert yaw.angle(1_500_000_000, 2_500_000_000) == pytest.approx(-0.1)  # last sample held forward
  assert yaw.angle(2_000_000_000, 1_000_000_000) == 0.0


def test_rotate_for_yaw_sign_and_magnitude():
  # the car yaws left 0.1 rad: a fixed point straight ahead is now to the right (y < 0), a point on the left comes ahead
  x, y = rotate_for_yaw(20.0, 0.0, 0.1)
  assert (x, y) == pytest.approx((20.0 * math.cos(0.1), -20.0 * math.sin(0.1)))
  assert y == pytest.approx(-1.9967, abs=1e-3)
  x, y = rotate_for_yaw(0.0, 5.0, math.pi / 2)  # a quarter turn left: the point on the left is straight ahead
  assert (x, y) == pytest.approx((5.0, 0.0), abs=1e-9)
  assert rotate_for_yaw(12.0, -3.0, 0.0) == (12.0, -3.0)
  for angle in (0.3, -0.7):
    x, y = rotate_for_yaw(12.0, -3.0, angle)
    assert math.hypot(x, y) == pytest.approx(math.hypot(12.0, -3.0))


def test_xcp_points_are_rotated_by_the_yaw_since_they_were_read(tmp_path):
  path = tmp_path / "xcp.json"
  # one point read 0.4 s before the file was written (age_lat), file written 0.1 s before "now"
  path.write_text(json.dumps({"mono_ns": 10_000_000_000, "points": [[20.0, 0.0, 0.0, 0.4, 5]]}))
  dots = RadarPointsRenderer(7.0, 9.0, xcp_path=path)
  # modelV2 orientationRate.z > 0 is a RIGHT turn (z down); the history is left-positive
  yaw_rate_left = 0.2
  sm = FakeSm([])
  sm.recv_frame["modelV2"] = 1
  sm["modelV2"] = NS(orientationRate=NS(z=[-yaw_rate_left]))
  dict.__setitem__(sm, "modelV2", NS(orientationRate=NS(z=[-yaw_rate_left])))
  dots._update_yaw(sm, 9_000_000_000)
  ids, pts = dots._raw_xcp(10_100_000_000)
  angle = yaw_rate_left * 0.5  # 0.4 s lateral age + 0.1 s file age
  assert ids == [5]
  np.testing.assert_allclose(pts, [rotate_for_yaw(20.0, 0.0, angle)])
  assert pts[0, 1] < 0.0  # left turn: the fixed point moves to the right
  # no yaw available: no rotation
  assert RadarPointsRenderer(7.0, 9.0, xcp_path=path)._raw_xcp(10_100_000_000)[1].tolist() == [[20.0, 0.0]]


def _track(tracker, t_ms, cam, xcp):
  """cam / xcp: {id: (d, y)}. Returns (purple cam ids, red cam ids, blue xcp ids); anything else is not drawn."""
  c_ids, x_ids = list(cam), list(xcp)
  purple, red, blue = tracker.update(c_ids, _a(*[cam[i] for i in c_ids]), _a(*[xcp[i] for i in x_ids]), t_ms * 1_000_000)
  return {c_ids[i] for i in purple}, {c_ids[i] for i in red}, {x_ids[j] for j in blue}


GRACE_MS = FUSE_GRACE_NS // 1_000_000


def test_confirm_needs_a_point_in_the_gate_and_far_range_uses_the_fractional_gate():
  tr = ConfirmTracker()
  # at 20 m the point is expected at 21.6 m (8 % range bias), gate 4 m
  assert _track(tr, 0, {1: (20.0, 0.0)}, {9: (26.0, 0.0)}) == (set(), {1}, {9})  # 4.4 m > 4 m gate
  assert _track(tr, 50, {1: (20.0, 0.0)}, {9: (21.0, 1.5)}) == (set(), {1}, {9})  # 1.5 m > 1.0 m lateral gate
  assert _track(tr, 100, {1: (20.0, 0.5)}, {9: (21.0, 0.0)}) == ({1}, set(), set())
  # at 60 m: expected 64.8 m, gate 0.12 * 60 = 7.2 m, so 57.6..72.0 m
  assert _track(ConfirmTracker(), 0, {1: (60.0, 0.0)}, {9: (71.5, 0.0)})[0] == {1}
  assert _track(ConfirmTracker(), 0, {1: (60.0, 0.0)}, {9: (72.5, 0.0)})[0] == set()
  assert _track(ConfirmTracker(), 0, {1: (60.0, 0.0)}, {9: (57.0, 0.0)})[0] == set()  # inside 60 +- 7.2, but short of the bias
  # a camera row that is not finite never confirms
  assert _track(ConfirmTracker(), 0, {1: (np.nan, 0.0), 2: (30.0, np.inf)}, {9: (30.0, 0.0)}) == (set(), {1, 2}, {9})


def test_many_points_confirm_one_camera_object_without_hopping():
  tr = ConfirmTracker()
  # two reflections of one car: both support it, neither is drawn
  assert _track(tr, 0, {1: (30.0, 0.0)}, {9: (30.5, 0.0), 10: (31.0, 0.3)}) == ({1}, set(), set())
  # either one alone keeps it purple
  assert _track(tr, 50, {1: (30.0, 0.0)}, {10: (31.0, 0.3)}) == ({1}, set(), set())
  assert _track(tr, 100, {1: (30.0, 0.0)}, {9: (30.5, 0.0)}) == ({1}, set(), set())
  # a point outside the keep gate is another object and stays blue
  assert _track(tr, 150, {1: (30.0, 0.0)}, {9: (30.5, 0.0), 11: (30.0, 3.5)}) == ({1}, set(), {11})


def test_keep_gate_holds_and_hides_supporting_points():
  tr = ConfirmTracker()
  # 37.0 m is 4.6 m from the expected 32.4 m: outside the 4 m confirm gate, so the object stays red and the point blue
  assert _track(tr, 0, {1: (30.0, 0.0)}, {9: (37.0, 0.0)}) == (set(), {1}, {9})
  assert _track(tr, 50, {1: (30.0, 0.0)}, {9: (31.0, 0.0)}) == ({1}, set(), set())
  # once confirmed, the same point is inside the 8 m keep gate: it supports and is not drawn
  for t in range(100, 100 + 3 * GRACE_MS, 50):
    assert _track(tr, t, {1: (30.0, 0.0)}, {9: (37.0, 0.0)}) == ({1}, set(), set())


def test_grace_hold_then_red_and_the_points_reappear():
  tr = ConfirmTracker()
  _track(tr, 0, {1: (30.0, 0.0)}, {9: (31.0, 0.0)})
  assert _track(tr, 100, {1: (30.0, 0.0)}, {9: (42.0, 0.0)}) == ({1}, set(), {9})  # the point left the keep gate: still purple
  assert _track(tr, GRACE_MS, {1: (30.0, 0.0)}, {})[0] == {1}  # still inside the grace
  assert _track(tr, GRACE_MS + 50, {1: (30.0, 0.0)}, {9: (42.0, 0.0)}) == (set(), {1}, {9})
  # back inside the gate: purple again
  assert _track(tr, GRACE_MS + 100, {1: (30.0, 0.0)}, {9: (30.5, 0.0)}) == ({1}, set(), set())


def test_confirming_changes_the_colour_only(tmp_path, monkeypatch):
  import sys
  calls = []
  rl = NS(Color=lambda *c: tuple(c), Vector2=lambda x, y: (x, y), draw_circle_v=lambda c, r, col: calls.append((c, r, col)))
  monkeypatch.setitem(sys.modules, "openpilot.starpilot.ui.raylib", NS(rl=rl))
  xcp_path = tmp_path / "xcp.json"
  view, sm, rect = _renderer_inputs()
  sm[SERVICE].points[:] = [NS(dRel=30.0, yRel=-40.0)]
  dots = RadarPointsRenderer(7.0, 9.0, xcp_path=xcp_path)
  dots.enabled = True

  def fills(now_ns):
    calls.clear()
    dots.draw(sm, view, rect, now_ns=now_ns)
    return [(c[0], c[1], c[2]) for c in calls if c[2] in (FILL_COLOR, XCP_FILL_COLOR, FUSED_FILL_COLOR)]

  red = fills(10_000_000_000)
  assert red == [((30.0, 40.0), 7.0, FILL_COLOR)]
  # an XCP point 1.5 m longer and 0.6 m to the side confirms it: same place, same size, purple; the point is not drawn
  xcp_path.write_text(json.dumps({"mono_ns": 10_000_000_000, "points": [[31.5, -40.6]]}))
  assert fills(10_050_000_000) == [((30.0, 40.0), 7.0, FUSED_FILL_COLOR)]
  # radar goes stale: after the grace it is red again, still at the camera position
  assert fills(10_050_000_000 + 1_000_000_000 + FUSE_GRACE_NS) == red


def test_same_car_shows_one_purple_dot():
  tr = ConfirmTracker()
  assert _track(tr, 0, {1: (30.0, 0.0)}, {9: (30.5, 0.0)}) == ({1}, set(), set())
  # a second camera object 2 m behind and 0.5 m over is the same car: not drawn; one 10 m on stays red
  assert _track(tr, 50, {1: (30.0, 0.0), 2: (32.0, 0.5), 3: (40.0, 0.0)}, {9: (29.0, -0.3)}) == ({1}, {3}, set())
  assert tr._state[2].since_ns is None  # hidden while unconfirmed (point 5.6 m short of its expected 34.6 m, gate 4 m)
  # the second one gets its own support (and is closer to it): the older confirmation keeps the purple, every frame
  for t in range(100, 1000, 50):
    assert _track(tr, t, {1: (30.0, 0.0), 2: (32.0, 0.5)}, {9: (32.0, 0.5)}) == ({1}, set(), set())
  # the first one leaves: the second, already confirmed, takes over at its own position
  assert _track(tr, 1000, {2: (32.0, 0.5)}, {9: (32.0, 0.5)}) == ({2}, set(), set())


def test_reused_track_id_starts_unconfirmed():
  tr = ConfirmTracker()
  _track(tr, 0, {1: (30.0, 0.0)}, {9: (30.5, 0.0)})
  assert _track(tr, 50, {}, {}) == (set(), set(), set())
  # gone for longer than the grace: a new object with the same id is red until radar confirms it
  assert _track(tr, 50 + GRACE_MS + 50, {1: (12.0, 3.0)}, {}) == (set(), {1}, set())
  # gone for less than the grace: still the same confirmed object
  tr = ConfirmTracker()
  _track(tr, 0, {1: (30.0, 0.0)}, {9: (30.5, 0.0)})
  _track(tr, 50, {}, {})
  assert _track(tr, 100, {1: (30.2, 0.0)}, {}) == ({1}, set(), set())


def test_two_frame_entry_option(monkeypatch):
  import openpilot.starpilot.ui.radar_points as rp
  monkeypatch.setattr(rp, "FUSE_ENTER_FRAMES", 2)
  tr = ConfirmTracker()
  assert _track(tr, 0, {1: (30.0, 0.0)}, {9: (30.5, 0.0)}) == (set(), {1}, {9})
  assert _track(tr, 50, {1: (30.0, 0.0)}, {}) == (set(), {1}, set())  # the streak resets
  assert _track(tr, 100, {1: (30.0, 0.0)}, {9: (30.5, 0.0)})[0] == set()
  assert _track(tr, 150, {1: (30.0, 0.0)}, {9: (30.5, 0.0)})[0] == {1}
