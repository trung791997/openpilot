"""Radar points on the road view: one dot per object, colored by which source saw it.

red = camera objects (radarTracks), blue = radar (XCP Array B overlay), purple = a camera object that XCP radar
confirms (ConfirmTracker): drawn exactly where the red dot would be, so confirming changes only the colour. The XCP
points that confirm it and other camera objects on the same car are not drawn. XCP points are rotated by the ego yaw
since they were read (YawHistory).

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
FUSE_D_MIN_M = 4.0  # confirm gate around a camera object: |d_xcp - d*(1 + bias)| <= max(FUSE_D_MIN_M, FUSE_D_FRAC * d),
FUSE_D_FRAC = 0.12  # |dy| <= FUSE_Y_M
FUSE_Y_M = 1.0
FUSE_XCP_RANGE_BIAS_FRAC = 0.08  # XCP range of a moving car reads ~8 % longer than the camera object's; removed before gating
FUSE_KEEP_SCALE = 2.0  # a confirmed camera object stays confirmed while any XCP point is inside this multiple of the gate
FUSE_GRACE_NS = 1_000_000_000  # ...and for this long after its last supporting point (XCP points refresh every 0.2-0.7 s)
FUSE_ENTER_FRAMES = 1  # draws in a row with a point inside the gate before purple
SAME_OBJECT_D_M = 3.0  # an unconfirmed (or later-confirmed) camera object this close to a purple one is not drawn
SAME_OBJECT_Y_M = 1.0
# Replay evidence (route 10b, 49.1 min, offline UI sim, 2026-11-23). Range offset, same-lane pairs (|dy| <= 1 m), signed
# d_xcp - d_cam: grows with range for moving cars (median ~0.04 d at 5-15 m, ~0.09-0.11 d at 15-60 m), ~0 for stopped
# objects and at standstill; not the 3.39 m Array B trailer offset (the UI points never apply it) and not lag
# (corr with vRel * kin age -0.04). One 0.08 d term fits better than none or a moving-only term. Sweep (whole route;
# rb = frames with an unconfirmed red that has a blue within |dy| <= 1 m and 3 * max(2, 0.08 d)):
#   2 m/0.08, no bias, 0.6 s (previous): flips/min 7.68, rb 19.6 %, wrong-car proxy 0.12 %
#   4 m/0.12, 0.08 bias, 1.0 s (this):    flips/min 6.28, rb  2.5 %, wrong-car proxy 0.49 %
#   5 m/0.15, no bias, 1.0 s: 6.15 / 4.0 % / 0.64 %; 5 m/0.15, bias, 1.0 s: 6.38 / 1.2 % / 0.74 %
# The bias rows raise the wrong-car proxy in the 15 s two-lead window (1.7 -> 5.9 %). Entering after 2 frames gave more
# flips (9.11 vs 7.68 at the old gate), so 1. Same-object gate 4 m / 1.3 m changed nothing; 2 m / 0.7 m left a red dot
# next to a purple one in 13 frames.
YAW_HISTORY_NS = 3_000_000_000
YAW_FROM_MEASUREMENT = True  # rotate over the point's lateral age as well as the file age


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


class _Fusion:
  __slots__ = ("since_ns", "support_ns", "seen_ns", "streak")

  def __init__(self, now_ns: int):
    self.since_ns: int | None = None  # confirmed since (None: not confirmed)
    self.support_ns = now_ns
    self.seen_ns = now_ns
    self.streak = 0


def fuse_gates(cam: np.ndarray, xcp: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """(enter, keep): camera rows x XCP rows, True where the point is inside the confirm / keep gate of the object."""
  gate_d = np.maximum(FUSE_D_MIN_M, FUSE_D_FRAC * cam[:, 0])[:, None]
  dd = np.abs(xcp[None, :, 0] - cam[:, None, 0] * (1.0 + FUSE_XCP_RANGE_BIAS_FRAC))
  dy = np.abs(cam[:, None, 1] - xcp[None, :, 1])
  enter = (dd <= gate_d) & (dy <= FUSE_Y_M)  # NaN rows compare False: never confirm
  keep = (dd <= FUSE_KEEP_SCALE * gate_d) & (dy <= FUSE_KEEP_SCALE * FUSE_Y_M)
  return enter, keep


def _same_object(a: np.ndarray, b: np.ndarray) -> bool:
  return abs(a[0] - b[0]) < SAME_OBJECT_D_M and abs(a[1] - b[1]) < SAME_OBJECT_Y_M


class ConfirmTracker:
  """Purple = a camera object that XCP radar confirms; state is kept per camera trackId.

  A camera object is confirmed after FUSE_ENTER_FRAMES draws with an XCP point inside its gate. It stays confirmed
  while any point is inside FUSE_KEEP_SCALE times the gate, and for FUSE_GRACE_NS after the last one. Any number of
  points may support one object, so there is no pairing to hop between reflections. Confirming changes the colour
  only: purple is drawn at the camera position. Points supporting a confirmed object are not drawn; camera objects
  within SAME_OBJECT_* of a purple one are not drawn (the longest-confirmed purple wins). An id unseen for longer
  than the grace starts again unconfirmed.
  """

  def __init__(self):
    self._state: dict[int, _Fusion] = {}

  def update(self, cam_ids, cam: np.ndarray, xcp: np.ndarray, now_ns: int):
    """Returns (purple cam indices, red cam indices, blue xcp indices). Indices in neither list are not drawn."""
    cam = np.asarray(cam, dtype=np.float64).reshape(-1, 2)
    xcp = np.asarray(xcp, dtype=np.float64).reshape(-1, 2)
    enter, keep = fuse_gates(cam, xcp)
    self._state = {c: s for c, s in self._state.items() if now_ns - s.seen_ns <= FUSE_GRACE_NS}
    confirmed = []
    for i, cid in enumerate(cam_ids):
      s = self._state.setdefault(int(cid), _Fusion(now_ns))
      s.seen_ns = now_ns
      if s.since_ns is not None:
        if keep[i].any():
          s.support_ns = now_ns
        elif now_ns - s.support_ns > FUSE_GRACE_NS:
          s.since_ns, s.streak = None, 0
      if s.since_ns is None:
        s.streak = s.streak + 1 if enter[i].any() else 0
        if s.streak >= FUSE_ENTER_FRAMES:
          s.since_ns = s.support_ns = now_ns
      if s.since_ns is not None:
        confirmed.append(i)
    purple: list[int] = []
    for i in sorted(confirmed, key=lambda i: (self._state[int(cam_ids[i])].since_ns, int(cam_ids[i]))):
      if not any(_same_object(cam[i], cam[k]) for k in purple):
        purple.append(i)
    confirmed_set = set(confirmed)
    red = [i for i in range(len(cam)) if i not in confirmed_set and not any(_same_object(cam[i], cam[k]) for k in purple)]
    hidden_x = keep[confirmed].any(axis=0) if confirmed else np.zeros(len(xcp), dtype=bool)
    blue = [j for j in range(len(xcp)) if not hidden_x[j]]
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
    self._tracker = ConfirmTracker()

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
    _, xcp = self._raw_xcp(now)
    purple, red, blue = self._tracker.update(cam_ids, cam, xcp, now)
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
