"""Live road traffic for the head unit's dash map, one zoom-14 tile at a time (the 'q' datagrams of hu_feed.py).

Mapbox Traffic v1 vector tiles (layer "traffic", LineString/MultiLineString, property "congestion" of "low",
"moderate", "heavy" or "severe") are fetched for the 3x3 tiles around the car while a head unit that sent
"tq": 1 in its hello is being served, cut down to the congested lines and packed into the payload below.

Payload layout (all integers little endian, the same as road_tiles' "<4sBB" header and "<i4" coordinates):

  offset 0   4 bytes  magic b"SPTQ"
  offset 4   u8       format version, 1
  offset 5   u8       reserved, 0
  offset 6   u32      fetched_unix_seconds: wall-clock time of the Mapbox fetch
  offset 10  ...      zlib-compressed body (zlib.compress, level 6), which inflates to:
      u16 line_count
      line_count times:
        u8   level          2 moderate, 3 heavy, 4 severe (route_engine's 0-4 congestion scale; low/unknown never sent)
        u16  point_count    at least 2
        point_count times:
          i32 dx, i32 dy    the point minus the previous point of this line (the first point minus (0, 0)), in
                            road tile units: x right, y down, 0..4096 across the tile, buffered points past the
                            edge kept (negative or above 4096), each absolute coordinate rounded and clipped to
                            +-32767 before differencing, as road_tiles.encode_road_tile does
  A tile with no congested roads is sent with line_count 0 so the head unit clears what it drew before.

The head unit gets the payload as 'q' datagrams {"k": "x/y", "i", "n", "b": base64 of 1000-byte pieces}, chunked
the same as the 'm' road tile.
"""
from __future__ import annotations

import collections
import struct
import threading
import time
import zlib
from collections.abc import Callable

import numpy as np

from openpilot.starpilot.navigation.road_tiles import (
  _COORD_LIMIT, DATA_ZOOM, EXTENT, MAX_DOWNLOAD_BYTES, TileFormatError, _fields, _lines, _packed,
)

TRAFFIC_URL = "https://api.mapbox.com/v4/mapbox.mapbox-traffic-v1/{z}/{x}/{y}.vector.pbf"
MAGIC = b"SPTQ"
VERSION = 1
_HEADER = struct.Struct("<4sBBI")
LEVELS = {"low": 1, "moderate": 2, "heavy": 3, "severe": 4}
MIN_LEVEL = 2
MAX_LINES = 65535
MAX_POINTS = 65535

REFRESH_S = 300.        # per tile
MAX_FETCHES = 9         # per REFRESH_S, in steady state
BACKOFF_FIRST_S = 60.
BACKOFF_MAX_S = 900.
REQUEST_TIMEOUT = 10.
WANT_FRESH_S = 5.       # hu_feed must have asked this recently for a fetch to go out
LOG_EVERY_S = 60.
TOKEN_EVERY_S = 60.
KEEP_TILES = 25


def decode_traffic_mvt(data: bytes) -> list[tuple[int, np.ndarray]]:
  """(level, points) for each congested (moderate, heavy, severe) line of a Mapbox Traffic v1 tile, the points
  in 0..EXTENT tile units (scaled from the layer's extent, buffered points kept), as road lines are."""
  if not data:
    return []
  if data[:2] == b"\x1f\x8b":
    inflater = zlib.decompressobj(16 + zlib.MAX_WBITS)
    try:
      data = inflater.decompress(data, 4 * MAX_DOWNLOAD_BYTES)
    except zlib.error as error:
      raise TileFormatError("corrupt gzip tile") from error
    if inflater.unconsumed_tail:
      raise TileFormatError("vector tile too large")
  out: list[tuple[int, np.ndarray]] = []
  for field, wire, layer in _fields(data):
    if field != 3 or wire != 2:
      continue
    name, keys, values, features, extent = "", [], [], [], EXTENT
    for lf, lw, value in _fields(bytes(layer)):
      if lf == 1 and lw == 2:
        name = bytes(value).decode("utf-8", "replace")
        if name != "traffic":
          break
      elif lf == 2 and lw == 2:
        features.append(value)
      elif lf == 3 and lw == 2:
        keys.append(bytes(value).decode("utf-8", "replace"))
      elif lf == 4 and lw == 2:
        text = None
        for vf, vw, item in _fields(bytes(value)):
          if vf == 1 and vw == 2:
            text = bytes(item).decode("utf-8", "replace")
        values.append(text)
      elif lf == 5 and lw == 0:
        extent = value
    if name != "traffic" or extent <= 0 or "congestion" not in keys:
      continue
    scale = EXTENT / extent
    key_index = keys.index("congestion")
    for feature in features:
      tags, kind, geometry = [], 0, []
      for ff, fw, value in _fields(bytes(feature)):
        if ff == 2 and fw == 2:
          tags = _packed(value)
        elif ff == 3 and fw == 0:
          kind = value
        elif ff == 4 and fw == 2:
          geometry = _packed(value)
      if kind != 2:
        continue
      level = 0
      for k in range(0, len(tags) - 1, 2):
        if tags[k] == key_index and tags[k + 1] < len(values):
          level = LEVELS.get(values[tags[k + 1]] or "", 0)
      if level < MIN_LEVEL:
        continue
      for line in _lines(geometry):
        points = np.asarray(line, dtype=np.float64)
        if scale != 1.0:
          points *= scale
        out.append((level, points))
  return out


def encode_traffic_tile(lines: list[tuple[int, np.ndarray]], fetched: int) -> bytes:
  """The payload (layout in the module docstring) for (level, points) lines; levels outside 2..4 and lines
  under two points are dropped."""
  body = bytearray()
  kept = 0
  for level, points in lines:
    if not MIN_LEVEL <= int(level) <= 4 or kept >= MAX_LINES:
      continue
    coords = np.clip(np.rint(np.asarray(points, dtype=np.float64)), -_COORD_LIMIT, _COORD_LIMIT).astype(np.int32)[:MAX_POINTS]
    if len(coords) < 2:
      continue
    deltas = coords.copy()
    deltas[1:] -= coords[:-1]
    body += struct.pack("<BH", int(level), len(coords)) + deltas.astype("<i4").tobytes()
    kept += 1
  return _HEADER.pack(MAGIC, VERSION, 0, int(fetched) & 0xFFFFFFFF) + zlib.compress(struct.pack("<H", kept) + bytes(body), 6)


def decode_traffic_payload(raw: bytes) -> tuple[int, list[tuple[int, np.ndarray]]]:
  """(fetched_unix_seconds, [(level, absolute int points)]) from a payload; for tests and checking the Java side."""
  if len(raw) < _HEADER.size:
    raise TileFormatError("short traffic payload")
  magic, version, _, fetched = _HEADER.unpack_from(raw)
  if magic != MAGIC or version != VERSION:
    raise TileFormatError("not a traffic payload")
  body = zlib.decompress(raw[_HEADER.size:])
  (count,) = struct.unpack_from("<H", body)
  pos, lines = 2, []
  for _ in range(count):
    level, n = struct.unpack_from("<BH", body, pos)
    pos += 3
    deltas = np.frombuffer(body, dtype="<i4", count=2 * n, offset=pos).reshape(n, 2)
    pos += 8 * n
    lines.append((level, np.cumsum(deltas, axis=0)))
  if pos != len(body):
    raise TileFormatError("trailing bytes")
  return fetched, lines


def payload_of(mvt: bytes, fetched: int) -> bytes:
  return encode_traffic_tile(decode_traffic_mvt(mvt), fetched)


def fetch_mvt(key: str, token: str) -> bytes:
  """The raw Traffic v1 tile for "x/y" at zoom 14; b"" when there is none (204, 404). Raises on any failure."""
  import requests
  x, y = (int(part) for part in key.split("/"))
  response = requests.get(TRAFFIC_URL.format(z=DATA_ZOOM, x=x, y=y), params={"access_token": token},
                          timeout=REQUEST_TIMEOUT, stream=True)
  try:
    if response.status_code in (204, 404):
      return b""
    if response.status_code != 200:
      raise OSError(f"traffic tile HTTP {response.status_code}")  # never the URL: it carries the token
    data = b"".join(response.iter_content(64 * 1024))
    if len(data) > MAX_DOWNLOAD_BYTES:
      raise OSError("traffic tile too large")
    return data
  finally:
    response.close()


def read_token() -> str:
  try:
    from openpilot.starpilot.navigation.owner import NavigationOwner
    return str(NavigationOwner().read()["token"] or "")
  except Exception:
    return ""


class TrafficTiles:
  """Keeps the encoded traffic payloads of the wanted tiles fresh. hu_feed calls want() every tick while a tq
  head unit is served; a background thread (or poll(), in tests) does the fetching. Memory only."""

  def __init__(self, fetch: Callable[[str, str], bytes] = fetch_mvt, token: Callable[[], str] = read_token,
               clock: Callable[[], float] = time.monotonic, wall: Callable[[], float] = time.time,  # noqa: TID251 epoch seconds stamp the payload
               log: Callable[..., None] = lambda *_, **__: None):
    self._fetch, self._token_of, self.clock, self.wall, self.log = fetch, token, clock, wall, log
    self._lock = threading.Lock()
    self._wanted: list[str] = []
    self._wanted_at = -float("inf")
    self._payload: dict[str, tuple[int, bytes]] = {}  # key -> (generation, payload)
    self._generation = 0
    self._due: dict[str, float] = {}
    self._failures: dict[str, int] = {}
    self._attempts: collections.deque[float] = collections.deque()
    self._token = ""
    self._token_at = -float("inf")
    self._no_token_logged = False
    self._log_at = -float("inf")
    self._total = self._errors = 0
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None

  def want(self, keys: list[str]) -> None:
    """Called every hu_feed tick while a tq head unit is served: the tiles around the car, nearest first."""
    with self._lock:
      self._wanted, self._wanted_at = list(keys), self.clock()
    if self._thread is None:
      self._thread = threading.Thread(target=self._run, name="hu-traffic", daemon=True)
      self._thread.start()

  def idle(self) -> None:
    """No tq head unit to serve: nothing is fetched until want() is called again."""
    with self._lock:
      self._wanted = []

  def close(self) -> None:
    self._stop.set()
    if self._thread is not None:
      self._thread.join(timeout=2)

  def current(self, keys: list[str]) -> list[tuple[str, int, bytes]]:
    """(key, generation, payload) of each of keys that has a payload, in keys' order."""
    with self._lock:
      return [(key, *self._payload[key]) for key in keys if key in self._payload]

  def _run(self) -> None:
    while not self._stop.is_set():
      try:
        did = self.poll()
      except Exception as error:  # a background helper must never die quietly or raise into the feed
        self.log("hu_feed_traffic_error", error=f"{type(error).__name__}: {error}")
        did = False
      self._stop.wait(0.05 if did else 0.5)

  def _token_now(self, now: float) -> str:
    if now - self._token_at >= TOKEN_EVERY_S or not self._token:
      self._token, self._token_at = self._token_of() or "", now
    return self._token

  def poll(self) -> bool:
    """One scheduling pass: fetch the nearest due tile if the budget allows. True when a fetch was made."""
    now = self.clock()
    with self._lock:
      wanted = list(self._wanted) if now - self._wanted_at <= WANT_FRESH_S else []
    self._log_counts(now, bool(wanted))
    if not wanted:
      return False
    token = self._token_now(now)
    if not token:
      if not self._no_token_logged:
        self._no_token_logged = True
        self.log("hu_feed_traffic", status="no_token")
      return False
    self._no_token_logged = False
    while self._attempts and now - self._attempts[0] >= REFRESH_S:
      self._attempts.popleft()
    if len(self._attempts) >= MAX_FETCHES:
      return False
    key = next((k for k in wanted if self._due.get(k, -float("inf")) <= now), None)
    if key is None:
      return False
    self._attempts.append(now)
    self._total += 1
    try:
      raw = self._fetch(key, token)
      payload = payload_of(raw, int(self.wall()))
    except Exception:
      self._errors += 1
      count = self._failures.get(key, 0) + 1
      self._failures[key] = count
      self._due[key] = now + min(BACKOFF_FIRST_S * 2 ** (count - 1), BACKOFF_MAX_S)
      return True
    self._failures.pop(key, None)
    self._due[key] = now + REFRESH_S
    with self._lock:
      old = self._payload.get(key)
      if old is None or old[1][_HEADER.size:] != payload[_HEADER.size:]:
        self._generation += 1
        self._payload[key] = (self._generation, payload)
      if len(self._payload) > KEEP_TILES:
        for stale in [k for k in self._payload if k not in wanted][:len(self._payload) - KEEP_TILES]:
          del self._payload[stale]
          self._due.pop(stale, None)
    return True

  def _log_counts(self, now: float, active: bool) -> None:
    if active and now - self._log_at >= LOG_EVERY_S:
      self._log_at = now
      self.log("hu_feed_traffic", fetches=self._total, errors=self._errors, window=len(self._attempts), tiles=len(self._payload))
