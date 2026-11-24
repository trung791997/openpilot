"""Radar points on the road view: one dot per object, colored by which source saw it.

red = camera objects (radarTracks), blue = radar (XCP Array B overlay), purple = seen by both (drawn once, at the
camera position, which updates smoothly, when a camera object and an XCP point agree within FUSE_* gates in car space).
The pairing has hysteresis (PairTracker) and XCP points are rotated by the ego yaw since they were read (YawHistory).

Off by default. The choice is the optional `radarPoints` boolean in the onroad layout document, set from
"Radar Point Display" in the appearance settings. Display only: this never touches radar or control code.
"""

from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np

SERVICE = "radarTracks"
DOCUMENT_KEY = "radarPoints"
FILL_COLOR = (255, 40, 40, 230)
XCP_FILL_COLOR = (40, 110, 255, 230)
FUSED_FILL_COLOR = (170, 70, 255, 235)
OUTLINE_COLOR = (0, 0, 0, 170)
NO_POINTS = np.empty((0, 2), dtype=np.float64)
XCP_PATH = Path("/dev/shm/xcp_radar_points.json")
XCP_STALE_NS = 1_000_000_000
FUSE_D_MIN_M = 2.0
FUSE_D_FRAC = 0.08
FUSE_Y_M = 1.0
FUSE_KEEP_SCALE = 2.0  # an existing pair is kept while inside this multiple of the gates above
FUSE_GRACE_NS = 300_000_000  # a camera object stays purple this long after its pair drops
YAW_HISTORY_NS = 3_000_000_000
YAW_FROM_MEASUREMENT = True  # rotate over the point's lateral age as well as the file age


def match_fused(cam: np.ndarray, xcp: np.ndarray) -> list[tuple[int, int]]:
  """One-to-one greedy match of camera (d, y) rows to XCP (d, y) rows in car space.

  A pair is eligible when |dd| <= max(FUSE_D_MIN_M, FUSE_D_FRAC * d_xcp) and |dy| <= FUSE_Y_M. Eligible pairs are
  taken lowest normalized cost first (|dd| / d_gate + |dy| / FUSE_Y_M); used indices are skipped. Non-finite rows
  never match. Returns (cam_index, xcp_index) pairs.
  """
  cam = np.asarray(cam, dtype=np.float64).reshape(-1, 2)
  xcp = np.asarray(xcp, dtype=np.float64).reshape(-1, 2)
  pairs = []
  for i, (cd, cy) in enumerate(cam):
    if not (np.isfinite(cd) and np.isfinite(cy)):
      continue
    for j, (xd, xy) in enumerate(xcp):
      if not (np.isfinite(xd) and np.isfinite(xy)):
        continue
      d_gate = max(FUSE_D_MIN_M, FUSE_D_FRAC * xd)
      dd, dy = abs(cd - xd), abs(cy - xy)
      if dd <= d_gate and dy <= FUSE_Y_M:
        pairs.append((dd / d_gate + dy / FUSE_Y_M, i, j))
  pairs.sort()
  used_c, used_x, out = set(), set(), []
  for _, i, j in pairs:
    if i in used_c or j in used_x:
      continue
    used_c.add(i)
    used_x.add(j)
    out.append((i, j))
  return out


class YawHistory:
  """Ring buffer of ego yaw rate (rad/s, left turn positive) so a point can be rotated by the yaw since it was read."""

  def __init__(self, maxlen: int = 128):
    self._t: deque[int] = deque(maxlen=maxlen)
    self._w: deque[float] = deque(maxlen=maxlen)

  def add(self, t_ns: int, rate_left: float) -> None:
    if not math.isfinite(rate_left):
      return
    if self._t and t_ns <= self._t[-1]:
      return
    self._t.append(int(t_ns))
    self._w.append(float(rate_left))
    while self._t and self._t[-1] - self._t[0] > YAW_HISTORY_NS:
      self._t.popleft()
      self._w.popleft()

  def angle(self, t0_ns: int, t1_ns: int) -> float:
    """Yaw accumulated over [t0, t1] (rad, left positive). Each sample holds until the next; 0 with no history."""
    if not self._t or t1_ns <= t0_ns:
      return 0.0
    ts, ws = list(self._t), list(self._w)
    total = 0.0
    for i, w in enumerate(ws):
      start = ts[i] if i > 0 else t0_ns  # the first sample is held back to t0
      end = ts[i + 1] if i + 1 < len(ts) else t1_ns  # the last sample is held up to t1
      lo, hi = max(start, t0_ns), min(end, t1_ns)
      if hi > lo:
        total += w * (hi - lo) / 1e9
    return total


def rotate_for_yaw(d: float, y: float, angle_left: float) -> tuple[float, float]:
  """Car-frame (x forward, y left) position of a world-fixed point after the car yawed left by angle_left."""
  c, s = math.cos(angle_left), math.sin(angle_left)
  return d * c + y * s, -d * s + y * c


def keep_gate(xd: float) -> tuple[float, float]:
  return FUSE_KEEP_SCALE * max(FUSE_D_MIN_M, FUSE_D_FRAC * xd), FUSE_KEEP_SCALE * FUSE_Y_M


class PairTracker:
  """Camera/XCP pairing with hysteresis, so purple does not flicker.

  A pair from the last frame is kept while both ids are present and the XCP point is inside the keep gate (the make
  gate times FUSE_KEEP_SCALE). A camera object whose pair drops stays purple for FUSE_GRACE_NS. XCP points of a kept
  pair cannot be matched to another camera object. Ids are camera trackId and the logger's XCP id.
  """

  def __init__(self):
    self._pair: dict[int, int] = {}  # cam id -> xcp id
    self._purple_ns: dict[int, int] = {}  # cam id -> last time it was purple

  def reset(self) -> None:
    self._pair, self._purple_ns = {}, {}

  def update(self, cam_ids, cam: np.ndarray, xcp_ids, xcp: np.ndarray, now_ns: int):
    """Returns (purple cam indices, red cam indices, blue xcp indices)."""
    cam = np.asarray(cam, dtype=np.float64).reshape(-1, 2)
    xcp = np.asarray(xcp, dtype=np.float64).reshape(-1, 2)
    cam_index = {int(c): i for i, c in enumerate(cam_ids)}
    xcp_index = {int(x): j for j, x in enumerate(xcp_ids)}
    pairs: dict[int, int] = {}
    used_x: set[int] = set()
    for cid, xid in self._pair.items():
      i, j = cam_index.get(cid), xcp_index.get(xid)
      if i is None or j is None or j in used_x:
        continue
      gate_d, gate_y = keep_gate(xcp[j, 0])
      if abs(cam[i, 0] - xcp[j, 0]) <= gate_d and abs(cam[i, 1] - xcp[j, 1]) <= gate_y:
        pairs[cid] = xid
        used_x.add(j)
    free_c = [i for i, c in enumerate(cam_ids) if int(c) not in pairs]
    # XCP ids of dropped pairs stay out of the new matching while their camera object is in grace
    held_x = {self._pair[c] for c in self._pair if c not in pairs and now_ns - self._purple_ns.get(c, -10**18) <= FUSE_GRACE_NS}
    free_x = [j for j, x in enumerate(xcp_ids) if j not in used_x and int(x) not in held_x]
    if free_c and free_x:
      for a, b in match_fused(cam[free_c], xcp[free_x]):
        pairs[int(cam_ids[free_c[a]])] = int(xcp_ids[free_x[b]])
        used_x.add(free_x[b])
    for cid in pairs:
      self._purple_ns[cid] = now_ns
    purple = []
    for cid, i in cam_index.items():
      if cid in pairs or now_ns - self._purple_ns.get(cid, -10**18) <= FUSE_GRACE_NS:
        purple.append(i)
    live = set(cam_index)
    self._purple_ns = {c: t for c, t in self._purple_ns.items() if c in live and now_ns - t <= FUSE_GRACE_NS}
    self._pair = pairs
    purple_set = set(purple)
    red = [i for i in range(len(cam)) if i not in purple_set]
    blue = [j for j in range(len(xcp)) if j not in used_x]
    return sorted(purple), red, blue


def radar_points_enabled(document) -> bool:
  return type(document) is dict and document.get(DOCUMENT_KEY) is True


def project_radar_points(
  d_rel: np.ndarray,
  in_y: np.ndarray,
  path_x: np.ndarray,
  path_z: np.ndarray,
  transform: np.ndarray,
  path_offset_z: float,
  clip_bounds: tuple[float, float, float, float],
  rect_bounds: tuple[float, float, float, float],
) -> np.ndarray:
  """Project radar points into screen space, preserving source order.

  Each point sits on the model path's height at its distance. Points that land outside clip_bounds are dropped;
  the rest are clamped into rect_bounds.
  """
  if d_rel.size == 0 or path_x.size == 0:
    return NO_POINTS

  finite = np.isfinite(d_rel) & np.isfinite(in_y)
  if not np.any(finite):
    return NO_POINTS

  d_rel = d_rel[finite]
  in_y = in_y[finite]

  path_indices = np.searchsorted(path_x, d_rel, side="right") - 1
  path_indices = np.maximum(path_indices, 0)

  line_z = np.zeros(d_rel.shape, dtype=np.float64)
  valid_z = path_indices < path_z.size
  if np.any(valid_z):
    line_z[valid_z] = path_z[path_indices[valid_z]]

  in_z = line_z + float(path_offset_z)

  t = transform.astype(np.float64, copy=False)
  point_w = t[2, 0] * d_rel + t[2, 1] * in_y + t[2, 2] * in_z
  valid_w = np.abs(point_w) >= 1e-6

  x = np.zeros_like(d_rel)
  y = np.zeros_like(d_rel)
  x_num = t[0, 0] * d_rel + t[0, 1] * in_y + t[0, 2] * in_z
  y_num = t[1, 0] * d_rel + t[1, 1] * in_y + t[1, 2] * in_z
  np.divide(x_num, point_w, out=x, where=valid_w)
  np.divide(y_num, point_w, out=y, where=valid_w)

  clip_x, clip_y, clip_xmax, clip_ymax = clip_bounds
  visible = (
    valid_w
    & (x >= clip_x) & (x <= clip_xmax)
    & (y >= clip_y) & (y <= clip_ymax)
  )
  if not np.any(visible):
    return NO_POINTS

  rect_x, rect_y, rect_xmax, rect_ymax = rect_bounds
  return np.column_stack((
    np.clip(x[visible], rect_x, rect_xmax),
    np.clip(y[visible], rect_y, rect_ymax),
  ))


class RadarPointsRenderer:
  """Draws the dots for one model renderer. `enabled` is set by the road view before each frame.

  rect_relative: the renderer's transform gives coordinates relative to its rect (compact UI) rather than the screen.
  clip_margin: how far outside the view a point may land and still be drawn, pinned to the view's edge. With 0,
  points off the view are dropped.
  """

  def __init__(
    self,
    radius: float,
    outline_radius: float,
    *,
    rect_relative: bool = False,
    clip_margin: float = 0.0,
    xcp_path: Path | None = XCP_PATH,
  ):
    self.enabled = False
    self._radius = radius
    self._outline_radius = outline_radius
    self._rect_relative = rect_relative
    self._clip_margin = clip_margin
    self._xcp_path = Path(xcp_path) if xcp_path is not None else None
    self._key = None
    self._points = NO_POINTS
    self._xcp_file_sig: tuple[int, int] | None = None
    self._xcp_raw_mono_ns = 0
    self._xcp_raw_points: list[tuple[float, float, float, float, int]] = []
    self._xcp_key = None
    self._xcp_points = NO_POINTS
    self._yaw = YawHistory()
    self._yaw_frame = None
    self._tracker = PairTracker()

  def _project_view_points(self, d_rel: np.ndarray, in_y: np.ndarray, path: np.ndarray, renderer, rect) -> np.ndarray:
    origin_x, origin_y = (0.0, 0.0) if self._rect_relative else (float(rect.x), float(rect.y))
    view = (origin_x, origin_y, origin_x + float(rect.width), origin_y + float(rect.height))
    margin = self._clip_margin
    clip = (view[0] - margin, view[1] - margin, view[2] + margin, view[3] + margin)
    screen = project_radar_points(
      d_rel, in_y, path[:, 0], path[:, 2],
      renderer._car_space_transform, float(renderer._path_offset_z), clip, view,
    )
    if self._rect_relative and screen.size:
      screen = screen + (float(rect.x), float(rect.y))
    return screen

  def screen_points(self, sm, renderer, rect) -> np.ndarray:
    """Absolute screen positions of the current radar points; empty when off, stale or invalid."""
    if not self.enabled or not (sm.seen[SERVICE] and sm.alive[SERVICE] and sm.valid[SERVICE]):
      self._key = None
      return NO_POINTS
    path = renderer._path.raw_points
    if path.size == 0:
      self._key = None
      return NO_POINTS

    transform = renderer._car_space_transform
    key = (sm.recv_frame[SERVICE], sm.recv_frame["modelV2"], transform.tobytes(), float(renderer._path_offset_z),
           float(rect.x), float(rect.y), float(rect.width), float(rect.height))
    if key == self._key:
      return self._points

    points = sm[SERVICE].points
    count = len(points)
    screen = self._project_view_points(
      np.fromiter((float(point.dRel) for point in points), dtype=np.float64, count=count),
      np.fromiter((float(-point.yRel) for point in points), dtype=np.float64, count=count),
      path, renderer, rect,
    )
    self._key, self._points = key, screen
    return screen

  def _read_xcp_snapshot(self, now_ns: int) -> tuple[tuple[int, int] | None, list[tuple[float, float, float, float, int]]]:
    if self._xcp_path is None:
      return None, []
    try:
      st = self._xcp_path.stat()
    except OSError:
      self._xcp_file_sig = None
      self._xcp_raw_mono_ns = 0
      self._xcp_raw_points = []
      return None, []
    sig = (int(getattr(st, "st_ino", 0)), int(st.st_mtime_ns))
    if sig != self._xcp_file_sig:
      try:
        data = json.loads(self._xcp_path.read_bytes())
        raw_mono = int(data["mono_ns"])
        raw_pts = []
        for k, p in enumerate(data.get("points", ())):
          if len(p) < 2:
            continue
          # [d, y] (old files) or [d, y, age_kin_s, age_lat_s, id]; without an id the index stands in
          full = len(p) >= 5
          raw_pts.append((float(p[0]), float(p[1]), float(p[2]) if full else 0.0, float(p[3]) if full else 0.0,
                          int(p[4]) if full else -1 - k))
        self._xcp_file_sig = sig
        self._xcp_raw_mono_ns = raw_mono
        self._xcp_raw_points = raw_pts
      except Exception:
        return None, []
    if not self._xcp_raw_points or (now_ns - self._xcp_raw_mono_ns) > XCP_STALE_NS or self._xcp_raw_mono_ns > now_ns + 500_000_000:
      return None, []
    return sig, self._xcp_raw_points

  def xcp_screen_points(self, sm, renderer, rect, *, now_ns: int | None = None) -> np.ndarray:
    """Absolute screen positions of live XCP Array B points; empty when off, missing or stale."""
    if not self.enabled:
      self._xcp_key = None
      return NO_POINTS
    path = renderer._path.raw_points
    if path.size == 0:
      self._xcp_key = None
      return NO_POINTS
    now = time.monotonic_ns() if now_ns is None else int(now_ns)
    sig, raw_pts = self._read_xcp_snapshot(now)
    if sig is None:
      self._xcp_key = None
      return NO_POINTS

    transform = renderer._car_space_transform
    key = (sig, sm.recv_frame.get("modelV2"), transform.tobytes(), float(renderer._path_offset_z),
           float(rect.x), float(rect.y), float(rect.width), float(rect.height))
    if key == self._xcp_key:
      return self._xcp_points

    screen = self._project_view_points(
      np.fromiter((pt[0] for pt in raw_pts), dtype=np.float64, count=len(raw_pts)),
      np.fromiter((-pt[1] for pt in raw_pts), dtype=np.float64, count=len(raw_pts)),
      path, renderer, rect,
    )
    self._xcp_key, self._xcp_points = key, screen
    return screen

  def _raw_camera(self, sm) -> tuple[list[int], np.ndarray]:
    if not (sm.seen[SERVICE] and sm.alive[SERVICE] and sm.valid[SERVICE]):
      return [], NO_POINTS
    points = sm[SERVICE].points
    ids = [int(getattr(p, "trackId", k)) for k, p in enumerate(points)]
    return ids, np.asarray([(float(p.dRel), float(p.yRel)) for p in points], dtype=np.float64).reshape(-1, 2)

  def _update_yaw(self, sm, now_ns: int) -> None:
    """Feed the yaw history from modelV2 (orientationRate.z[0] is positive for a right turn, so negate to left-positive)."""
    try:
      frame = sm.recv_frame.get("modelV2")
      if frame is None or frame == self._yaw_frame:
        return
      self._yaw_frame = frame
      rates = sm["modelV2"].orientationRate.z
      if not len(rates):
        return
      t = now_ns
      try:
        stamp = int(sm.logMonoTime["modelV2"])
        if 0 < now_ns - stamp < 1_000_000_000:
          t = stamp
      except Exception:
        pass
      self._yaw.add(t, -float(rates[0]))
    except Exception:
      pass

  def _raw_xcp(self, now_ns: int) -> tuple[list[int], np.ndarray]:
    """XCP points in the car frame of now_ns: each is rotated by the ego yaw since it was read."""
    sig, raw_pts = self._read_xcp_snapshot(now_ns)
    if sig is None:
      return [], NO_POINTS
    file_ns = self._xcp_raw_mono_ns
    ids, rows = [], []
    for d, y, _, age_lat, xid in raw_pts:
      t0 = file_ns - int(age_lat * 1e9) if YAW_FROM_MEASUREMENT else file_ns
      d, y = rotate_for_yaw(d, y, self._yaw.angle(t0, now_ns))
      ids.append(xid)
      rows.append((d, y))
    return ids, np.asarray(rows, dtype=np.float64).reshape(-1, 2)

  def _project_raw(self, pts: np.ndarray, renderer, rect) -> np.ndarray:
    if not pts.size:
      return NO_POINTS
    return self._project_view_points(pts[:, 0], -pts[:, 1], renderer._path.raw_points, renderer, rect)

  def draw(self, sm, renderer, rect, *, now_ns: int | None = None) -> None:
    if not self.enabled:
      return
    from openpilot.starpilot.ui.raylib import rl
    if renderer._path.raw_points.size == 0:
      return
    now = time.monotonic_ns() if now_ns is None else int(now_ns)
    self._update_yaw(sm, now)
    cam_ids, cam = self._raw_camera(sm)
    xcp_ids, xcp = self._raw_xcp(now)
    purple, red, blue = self._tracker.update(cam_ids, cam, xcp_ids, xcp, now)
    small = (max(2.5, self._radius * 0.72), max(4.0, self._outline_radius * 0.72))
    outline = rl.Color(*OUTLINE_COLOR)
    layers = (
      (self._project_raw(cam[red], renderer, rect) if red else NO_POINTS, FILL_COLOR, self._radius, self._outline_radius),
      (self._project_raw(xcp[blue], renderer, rect) if blue else NO_POINTS, XCP_FILL_COLOR, *small),
      (self._project_raw(cam[purple], renderer, rect) if purple else NO_POINTS, FUSED_FILL_COLOR, self._radius, self._outline_radius),
    )
    for pts, fill_color, rad, out_rad in layers:
      if not pts.size:
        continue
      fill = rl.Color(*fill_color)
      for x, y in pts:
        center = rl.Vector2(float(x), float(y))
        rl.draw_circle_v(center, out_rad, outline)
        rl.draw_circle_v(center, rad, fill)
