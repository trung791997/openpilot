"""Road-line map tiles for the Android Auto map overlay.

The overlay draws roads as lines over the camera, so it needs road geometry,
not a picture of a map. Mapbox Vector Tiles carry that geometry; this module
keeps only the ``road`` layer and stores it in a small compact file per tile.
One stored zoom serves every display zoom (vectors scale without blurring),
and there is no light/dark pair: colors are chosen when drawing.

Pure functions only: no network, GPU or process state. ``navtilesd`` owns the
downloads and the renderer reads the files this module writes.
"""

from __future__ import annotations

import math
import struct
import zlib
from collections.abc import Iterable, Mapping, Sequence
import dataclasses
from dataclasses import dataclass

import numpy as np

DATA_ZOOM = 14
EXTENT = 4096
MVT_URL = "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/{z}/{x}/{y}.vector.pbf"
MAX_DOWNLOAD_BYTES = 4 * 1024 * 1024
MAX_STORED_BYTES = 2 * 1024 * 1024

# Draw order is the reverse of importance, so major roads end up on top.
MOTORWAY, MAJOR, STREET, LINK, MINOR = range(5)
CLASS_NAMES = ("motorway", "major", "street", "link", "minor")
_CLASS_OF = {
  "motorway": MOTORWAY, "trunk": MOTORWAY,
  "primary": MAJOR, "secondary": MAJOR,
  "tertiary": STREET, "street": STREET, "street_limited": STREET,
  "motorway_link": LINK, "trunk_link": LINK, "primary_link": LINK, "secondary_link": LINK, "tertiary_link": LINK,
  "service": MINOR, "track": MINOR,
}

MAGIC = b"SPRT"
# Version 2 (2026-10-30) adds a street name per line, for Peter's ask to see the street the car is on
# without a route and the names of nearby streets. Version 1 files (no names) still decode, and
# downgrade_road_tile() turns a version 2 file back into version 1 for head units that only read 1.
# Version 3 (2026-10-31) adds a flags byte per line, BRIDGE or TUNNEL from the vector tile's ``structure``,
# so the dash map can draw an overpass over the road under it (Peter). Older versions still decode (no
# flags), and downgrade_road_tile() makes a version 2 (names, no flags) or 1 copy for older head units.
# Version 4 (2026-10-31) adds the tile's traffic signal points after the classes, so the dash map can draw
# a small traffic-light icon at a signalised intersection like Google Maps (Peter). Mapbox Streets has no
# stop or give-way points; STOP is reserved for a future source. Older versions still decode (no points),
# and downgrade_road_tile() makes a version 3 (flags, no points) or older copy for older head units.
# Version 5 (2026-11-01) adds, for the dash map's close-up (Peter: Google Maps' immersive view and sidewalks),
# two bytes per line after the flags, the vector tile's ``layer`` (int8, the stacking order of bridges and
# tunnels) and ``lane_count`` (uint8, 0 unknown: Mapbox Streets v8 has it only in Japan), a ONEWAY flag bit,
# and after the points a table of walk lines: sidewalks, crossings and footways (class ``path``) and
# pedestrian streets (class ``pedestrian``), each with a kind byte. Older versions still decode (no layers,
# lanes or walks) and downgrade_road_tile() makes a version 4 or older copy for older head units.
# Version 6 (2026-11-01) adds, for Peter's railroad crossings, the point kind LEVEL_CROSSING (road layer point
# class ``level_crossing``: a road crossing a railway on the same level) and after the walks a table of rail
# lines in the walk table's layout, with a kind byte per line (MAJOR_RAIL, MINOR_RAIL, SERVICE_RAIL, plus
# RAIL_BRIDGE when the rail is on a bridge; rails in a tunnel are not kept). downgrade_road_tile() makes a
# version 5 or older copy without the rails and without the crossing points.
VERSION = 6
OLDEST_VERSION = 1
BRIDGE, TUNNEL, ONEWAY = 1, 2, 4  # line flags; ONEWAY from version 5
SIDEWALK, CROSSING, FOOTWAY, PEDESTRIAN = 1, 2, 3, 4  # walk kinds
# class path's types kept as walks; cycleways, trails, steps and indoor corridors are not drawn
_WALK_TYPE = {"sidewalk": SIDEWALK, "crossing": CROSSING, "footway": FOOTWAY, "path": FOOTWAY}
MAX_WALK_LINES = 16384
MAX_WALK_POINTS = 65535
SIGNAL, STOP, LEVEL_CROSSING = 1, 2, 3  # point kinds; LEVEL_CROSSING from version 6
_POINT_KIND = {"traffic_signals": SIGNAL, "level_crossing": LEVEL_CROSSING}
MAJOR_RAIL, MINOR_RAIL, SERVICE_RAIL = 1, 2, 3  # rail kinds (version 6): mainline, light rail/tram, yards/sidings
RAIL_BRIDGE = 0x80  # rail kind bit: the rail is on a bridge
_RAIL_KIND = {"major_rail": MAJOR_RAIL, "minor_rail": MINOR_RAIL, "service_rail": SERVICE_RAIL}
MAX_RAIL_LINES = 4096
MAX_RAIL_POINTS = 32767
MAX_POINTS = 4096
_STRUCTURE_FLAG = {"bridge": BRIDGE, "tunnel": TUNNEL}
_LAYER_LIMIT, _LANES_LIMIT = 15, 15  # Mapbox: 99.9999% of layers within -5..5
MAX_NAME_BYTES = 64
_HEADER = struct.Struct("<4sBB")
_CLASS_HEADER = struct.Struct("<BII")
_WALK_HEADER = struct.Struct("<II")
_COORD_LIMIT = 32767


class TileFormatError(ValueError):
  pass


@dataclass(frozen=True, slots=True, order=True)
class TileKey:
  z: int
  x: int
  y: int

  def path(self) -> str:
    return f"{self.z}/{self.x}/{self.y}.rt"


# ---------------------------------------------------------------- web mercator

def world_xy(latitude: float, longitude: float, zoom: int = DATA_ZOOM) -> tuple[float, float]:
  """Web Mercator position in tile units at ``zoom`` (tile x, tile y as floats)."""
  latitude = max(-85.05112878, min(85.05112878, latitude))
  n = float(1 << zoom)
  x = (longitude + 180.0) / 360.0 * n
  sin_lat = math.sin(math.radians(latitude))
  y = (0.5 - math.log((1.0 + sin_lat) / (1.0 - sin_lat)) / (4.0 * math.pi)) * n
  return x, y


def lat_lon(x: float, y: float, zoom: int = DATA_ZOOM) -> tuple[float, float]:
  n = float(1 << zoom)
  longitude = x / n * 360.0 - 180.0
  latitude = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
  return latitude, longitude


def meters_per_tile(latitude: float, zoom: int = DATA_ZOOM) -> float:
  return 40075016.686 * math.cos(math.radians(latitude)) / (1 << zoom)


def tiles_in_radius(latitude: float, longitude: float, radius_m: float, zoom: int = DATA_ZOOM) -> list[TileKey]:
  """Tiles whose square touches a circle, nearest first."""
  cx, cy = world_xy(latitude, longitude, zoom)
  reach = radius_m / max(1.0, meters_per_tile(latitude, zoom))
  limit = (1 << zoom) - 1
  keys = []
  for ty in range(max(0, int(cy - reach)), min(limit, int(cy + reach)) + 1):
    for tx in range(int(math.floor(cx - reach)), int(math.floor(cx + reach)) + 1):
      nx, ny = min(max(cx, tx), tx + 1), min(max(cy, ty), ty + 1)
      if (nx - cx) ** 2 + (ny - cy) ** 2 <= reach * reach:
        keys.append((math.hypot(tx + 0.5 - cx, ty + 0.5 - cy), TileKey(zoom, tx % (1 << zoom), ty)))
  return [key for _, key in sorted(keys)]


def corridor_tiles(points: Sequence[tuple[float, float]], radius_tiles: int = 1, zoom: int = DATA_ZOOM,
                   limit: int = 4000) -> list[TileKey]:
  """Tiles along a (latitude, longitude) polyline, in travel order."""
  seen: set[TileKey] = set()
  ordered: list[TileKey] = []
  previous = None
  wrap = 1 << zoom
  for latitude, longitude in points:
    point = world_xy(latitude, longitude, zoom)
    samples = [point]
    if previous is not None:
      steps = int(max(abs(point[0] - previous[0]), abs(point[1] - previous[1])) * 2)
      samples = [(previous[0] + (point[0] - previous[0]) * i / (steps + 1),
                  previous[1] + (point[1] - previous[1]) * i / (steps + 1)) for i in range(1, steps + 2)]
    previous = point
    for sx, sy in samples:
      for dy in range(-radius_tiles, radius_tiles + 1):
        for dx in range(-radius_tiles, radius_tiles + 1):
          ty = int(sy) + dy
          if 0 <= ty < wrap:
            key = TileKey(zoom, (int(sx) + dx) % wrap, ty)
            if key not in seen:
              seen.add(key)
              ordered.append(key)
              if len(ordered) >= limit:
                return ordered
  return ordered


# ---------------------------------------------------------------- protobuf / MVT

def _varint(data: bytes, pos: int) -> tuple[int, int]:
  result = shift = 0
  while True:
    if pos >= len(data):
      raise TileFormatError("truncated varint")
    byte = data[pos]
    pos += 1
    result |= (byte & 0x7F) << shift
    if byte < 0x80:
      return result, pos
    shift += 7
    if shift > 63:
      raise TileFormatError("varint too long")


def _fields(data: bytes):
  """(field number, wire type, value) for each field; length-delimited values are memoryviews."""
  pos, end = 0, len(data)
  view = memoryview(data)
  while pos < end:
    tag, pos = _varint(data, pos)
    field, wire = tag >> 3, tag & 7
    if wire == 0:
      value, pos = _varint(data, pos)
    elif wire == 2:
      length, pos = _varint(data, pos)
      if pos + length > end:
        raise TileFormatError("truncated field")
      value, pos = view[pos:pos + length], pos + length
    elif wire == 1:
      value, pos = view[pos:pos + 8], pos + 8
    elif wire == 5:
      value, pos = view[pos:pos + 4], pos + 4
    else:
      raise TileFormatError("unsupported wire type")
    if pos > end:
      raise TileFormatError("truncated field")
    yield field, wire, value


def _packed(data) -> list[int]:
  data = bytes(data)
  values, pos = [], 0
  while pos < len(data):
    value, pos = _varint(data, pos)
    values.append(value)
  return values


def _lines(geometry: list[int]) -> list[list[tuple[int, int]]]:
  """LineString parts from MVT geometry commands (zigzag deltas)."""
  lines: list[list[tuple[int, int]]] = []
  current: list[tuple[int, int]] = []
  x = y = i = 0
  while i < len(geometry):
    command, count = geometry[i] & 7, geometry[i] >> 3
    i += 1
    if command == 7:
      continue
    if command not in (1, 2) or i + 2 * count > len(geometry):
      raise TileFormatError("bad geometry command")
    for _ in range(count):
      dx, dy = geometry[i], geometry[i + 1]
      i += 2
      x += (dx >> 1) ^ -(dx & 1)
      y += (dy >> 1) ^ -(dy & 1)
      if command == 1:
        if len(current) >= 2:
          lines.append(current)
        current = [(x, y)]
      else:
        current.append((x, y))
  if len(current) >= 2:
    lines.append(current)
  return lines


def _points(geometry: list[int]) -> list[tuple[int, int]]:
  """Points from MVT point geometry (MoveTo commands only, zigzag deltas)."""
  points: list[tuple[int, int]] = []
  x = y = i = 0
  while i < len(geometry):
    command, count = geometry[i] & 7, geometry[i] >> 3
    i += 1
    if command != 1 or i + 2 * count > len(geometry):
      raise TileFormatError("bad point geometry")
    for _ in range(count):
      dx, dy = geometry[i], geometry[i + 1]
      i += 2
      x += (dx >> 1) ^ -(dx & 1)
      y += (dy >> 1) ^ -(dy & 1)
      points.append((x, y))
  return points


def decode_mvt_roads(data: bytes) -> dict[int, list[np.ndarray]]:
  """Road lines by class from a Mapbox Streets vector tile, in 0..EXTENT tile units."""
  return decode_mvt_named_roads(data)[0]


def decode_mvt_named_roads(data: bytes) -> tuple[dict[int, list[np.ndarray]], dict[int, list[str]]]:
  """Road lines by class plus a parallel list of street names ("" when the road has none).

  The name is the ``name`` tag, else ``ref`` (a route number such as "I 5"), so a numbered highway
  still gets a label."""
  roads, names, _ = decode_mvt_road_layer(data)
  return roads, names


def decode_mvt_road_layer(data: bytes) -> tuple[dict[int, list[np.ndarray]], dict[int, list[str]], dict[int, list[int]]]:
  """Road lines, names and flags by class, the three lists parallel (see decode_mvt_road_tile)."""
  roads, names, flags, _ = decode_mvt_road_tile(data)
  return roads, names, flags


def decode_mvt_road_tile(data: bytes) -> tuple[dict[int, list[np.ndarray]], dict[int, list[str]], dict[int, list[int]],
                                               list[tuple[float, float, int]]]:
  """Road lines, names and flags by class, the three lists parallel, and the tile's signal points
  (the version 4 content; see decode_mvt_road_detail)."""
  detail = decode_mvt_road_detail(data)
  return detail.roads, detail.names, detail.flags, detail.points


@dataclass
class MvtRoads:
  """The road layer as decode_mvt_road_detail reads it: per class parallel lists of lines, names, flags,
  layers and lane counts, the signal points, and the walk lines with their kinds (parallel)."""
  roads: dict[int, list[np.ndarray]]
  names: dict[int, list[str]]
  flags: dict[int, list[int]]
  layers: dict[int, list[int]]
  lanes: dict[int, list[int]]
  points: list[tuple[float, float, int]]
  walks: list[np.ndarray]
  walk_kinds: list[int]
  rails: list[np.ndarray] = dataclasses.field(default_factory=list)
  rail_kinds: list[int] = dataclasses.field(default_factory=list)

  def encode(self, version: int = VERSION) -> bytes:
    return encode_road_tile(self.roads, self.names, version=version, flags=self.flags, points=self.points,
                            layers=self.layers, lanes=self.lanes, walks=list(zip(self.walks, self.walk_kinds, strict=True)),
                            rails=list(zip(self.rails, self.rail_kinds, strict=True)))


def decode_mvt_road_detail(data: bytes) -> MvtRoads:
  """Road lines, names and flags by class, the three lists parallel, and the tile's signal points.

  The points are (x, y, kind) in 0..EXTENT tile units, from the road layer's point features of class
  ``traffic_signals`` (OSM highway=traffic_signals nodes). Points in the tile's buffer, outside
  0..EXTENT, are left to the neighbouring tile so a signal near an edge is not stored twice.

  The flags are BRIDGE or TUNNEL from the ``structure`` tag (Mapbox Streets v8: none, bridge, tunnel
  or ford), 0 otherwise, plus ONEWAY from ``oneway`` "true". The numeric ``layer`` (stacking order, clipped
  to +-15) and ``lane_count`` (0..15, 0 unknown) are kept per line too, for the close-up's raised overpasses
  and lane lines. Class ``path`` lines of type sidewalk, crossing, footway or path and class ``pedestrian``
  lines become walk lines (SIDEWALK, CROSSING, FOOTWAY, PEDESTRIAN), at most MAX_WALK_LINES and
  MAX_WALK_POINTS points a tile. Class ``major_rail``, ``minor_rail`` and ``service_rail`` lines become rail
  lines (RAIL_BRIDGE added on a bridge; rails in a tunnel, such as subways, are dropped), at most
  MAX_RAIL_LINES and MAX_RAIL_POINTS points, and ``level_crossing`` points become LEVEL_CROSSING points."""
  if data[:2] == b"\x1f\x8b":
    inflater = zlib.decompressobj(16 + zlib.MAX_WBITS)
    try:
      data = inflater.decompress(data, 4 * MAX_DOWNLOAD_BYTES)
    except zlib.error as error:
      raise TileFormatError("corrupt gzip tile") from error
    if inflater.unconsumed_tail:
      raise TileFormatError("vector tile too large")
  roads: dict[int, list[np.ndarray]] = {}
  names: dict[int, list[str]] = {}
  flags: dict[int, list[int]] = {}
  layers: dict[int, list[int]] = {}
  lanes: dict[int, list[int]] = {}
  signals: list[tuple[float, float, int]] = []
  walks: list[np.ndarray] = []
  walk_kinds: list[int] = []
  walk_points = 0
  rails: list[np.ndarray] = []
  rail_kinds: list[int] = []
  rail_points = 0
  for field, wire, layer in _fields(data):
    if field != 3 or wire != 2:
      continue
    layer = bytes(layer)
    name, keys, values, features, extent = "", [], [], [], EXTENT
    for lf, lw, value in _fields(layer):
      if lf == 1 and lw == 2:
        name = bytes(value).decode("utf-8", "replace")
        if name != "road":
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
          elif vf in (4, 5, 6) and vw == 0:  # int, uint, sint: layer and lane_count
            text = -((item + 1) >> 1) if item & 1 and vf == 6 else (item >> 1 if vf == 6 else item)
            if vf == 4 and text >= 1 << 63:
              text -= 1 << 64
          elif vf == 7 and vw == 0:  # bool: oneway
            text = bool(item)
          elif vf in (2, 3) and vw in (1, 5):  # float, double
            text = struct.unpack("<f" if vw == 5 else "<d", bytes(item))[0]
        values.append(text)
      elif lf == 5 and lw == 0:
        extent = value
    if name != "road" or extent <= 0:
      continue
    scale = EXTENT / extent
    class_key = keys.index("class") if "class" in keys else -1
    name_key = keys.index("name") if "name" in keys else -1
    ref_key = keys.index("ref") if "ref" in keys else -1
    structure_key = keys.index("structure") if "structure" in keys else -1
    type_key = keys.index("type") if "type" in keys else -1
    oneway_key = keys.index("oneway") if "oneway" in keys else -1
    layer_key = keys.index("layer") if "layer" in keys else -1
    lanes_key = keys.index("lane_count") if "lane_count" in keys else -1
    for feature in features:
      tags, kind, geometry = [], 0, []
      for ff, fw, value in _fields(bytes(feature)):
        if ff == 2 and fw == 2:
          tags = _packed(value)
        elif ff == 3 and fw == 0:
          kind = value
        elif ff == 4 and fw == 2:
          geometry = _packed(value)
      if kind == 1 and class_key >= 0:
        point_kind = 0
        for k in range(0, len(tags) - 1, 2):
          if tags[k] == class_key and tags[k + 1] < len(values):
            point_kind = _POINT_KIND.get(values[tags[k + 1]] or "", 0)
        if point_kind:
          for px, py in _points(geometry):
            px, py = px * scale, py * scale
            if 0 <= px < EXTENT and 0 <= py < EXTENT and len(signals) < MAX_POINTS:
              signals.append((px, py, point_kind))
        continue
      if kind != 2 or class_key < 0:
        continue
      road_class, road_name, road_ref, road_flags, road_layer, road_lanes = None, "", "", 0, 0, 0
      class_name, type_name, oneway = "", "", False
      for k in range(0, len(tags) - 1, 2):
        if tags[k + 1] >= len(values):
          continue
        value = values[tags[k + 1]]
        if tags[k] == class_key:
          class_name = value if isinstance(value, str) else ""
          road_class = _CLASS_OF.get(class_name)
        elif tags[k] == type_key:
          type_name = value if isinstance(value, str) else ""
        elif tags[k] == oneway_key:
          oneway = value is True or value == "true"
        elif tags[k] == layer_key:
          road_layer = _small_int(value, -_LAYER_LIMIT, _LAYER_LIMIT)
        elif tags[k] == lanes_key:
          road_lanes = _small_int(value, 0, _LANES_LIMIT)
        elif tags[k] == name_key:
          road_name = (values[tags[k + 1]] or "").strip()
        elif tags[k] == ref_key:
          road_ref = (values[tags[k + 1]] or "").strip()
        elif tags[k] == structure_key:
          road_flags = _STRUCTURE_FLAG.get(value if isinstance(value, str) else "", 0)
      walk = PEDESTRIAN if class_name == "pedestrian" else _WALK_TYPE.get(type_name, 0) if class_name == "path" else 0
      if walk:
        for line in _lines(geometry):
          if len(walks) >= MAX_WALK_LINES or walk_points + len(line) > MAX_WALK_POINTS:
            break
          points = np.asarray(line, dtype=np.float64)
          if scale != 1.0:
            points *= scale
          walks.append(points)
          walk_kinds.append(walk)
          walk_points += len(line)
        continue
      rail = _RAIL_KIND.get(class_name, 0)
      if rail:
        if road_flags == TUNNEL:
          continue
        if road_flags == BRIDGE:
          rail |= RAIL_BRIDGE
        for line in _lines(geometry):
          if len(rails) >= MAX_RAIL_LINES or rail_points + len(line) > MAX_RAIL_POINTS:
            break
          points = np.asarray(line, dtype=np.float64)
          if scale != 1.0:
            points *= scale
          rails.append(points)
          rail_kinds.append(rail)
          rail_points += len(line)
        continue
      if road_class is None:
        continue
      if oneway:
        road_flags |= ONEWAY
      for line in _lines(geometry):
        points = np.asarray(line, dtype=np.float64)
        if scale != 1.0:
          points *= scale
        roads.setdefault(road_class, []).append(points)
        names.setdefault(road_class, []).append(road_name or road_ref)
        flags.setdefault(road_class, []).append(road_flags)
        layers.setdefault(road_class, []).append(road_layer)
        lanes.setdefault(road_class, []).append(road_lanes)
  return MvtRoads(roads, names, flags, layers, lanes, signals, walks, walk_kinds, rails, rail_kinds)


def _small_int(value, low: int, high: int) -> int:
  """An int tag value (or its string form) clipped to low..high; anything else 0."""
  if isinstance(value, bool) or value is None:
    return 0
  try:
    number = int(float(value))
  except (TypeError, ValueError, OverflowError):
    return 0
  return max(low, min(high, number))


# ---------------------------------------------------------------- compact storage

def _clip_name(name: str) -> bytes:
  data = name.encode("utf-8")[:MAX_NAME_BYTES]
  return data.decode("utf-8", "ignore").encode("utf-8")


def encode_road_tile(roads: Mapping[int, Iterable[np.ndarray]], names: Mapping[int, Sequence[str]] | None = None,
                     version: int = VERSION, flags: Mapping[int, Sequence[int]] | None = None,
                     points: Sequence[tuple[float, float, int]] = (), layers: Mapping[int, Sequence[int]] | None = None,
                     lanes: Mapping[int, Sequence[int]] | None = None,
                     walks: Iterable[tuple[np.ndarray, int]] = (), rails: Iterable[tuple[np.ndarray, int]] = ()) -> bytes:
  """Delta-coded int16 polylines per class, zlib-compressed.

  Version 2 starts the body with the tile's distinct street names (each stored once, up to
  MAX_NAME_BYTES) and gives every line a 2-byte index into them (0 = unnamed), so a street cut into
  many lines costs its name once. Version 3 adds a byte per line after the name index, the line's
  flags (BRIDGE, TUNNEL). ``names[class]`` and ``flags[class]`` run parallel to ``roads[class]``.
  Version 4 ends the body with the tile's points: a 2-byte count, then per point int16 x, int16 y
  (tile units) and a kind byte (SIGNAL, STOP). Version 5 adds two bytes per line after the flags, the
  ``layers[class]`` stacking order (int8) and ``lanes[class]`` lane count (uint8, 0 unknown), keeps the
  ONEWAY flag bit (older versions mask flags to BRIDGE | TUNNEL), and ends the body with the walk lines:
  int32 line count and point count, the uint16 lengths, a kind byte per line (SIDEWALK, CROSSING,
  FOOTWAY, PEDESTRIAN), then the int32 x, y deltas, from ``walks`` as (points, kind) pairs. Version 6 ends
  the body with the rail lines from ``rails`` in the same layout (kind MAJOR_RAIL, MINOR_RAIL or SERVICE_RAIL,
  maybe with RAIL_BRIDGE), and keeps LEVEL_CROSSING points (older versions keep only SIGNAL and STOP)."""
  if not OLDEST_VERSION <= version <= VERSION:
    raise ValueError(f"unknown road tile version {version}")
  table: dict[bytes, int] = {}
  sections = bytearray()
  classes = 0
  for road_class in sorted(roads):
    class_names = list((names or {}).get(road_class, ()))
    class_flags = list((flags or {}).get(road_class, ()))
    class_layers = list((layers or {}).get(road_class, ()))
    class_lanes = list((lanes or {}).get(road_class, ()))
    flag_mask = 0xFF if version >= 5 else BRIDGE | TUNNEL
    pairs = [(np.clip(np.rint(line), -_COORD_LIMIT, _COORD_LIMIT).astype(np.int32),
              class_names[i] if i < len(class_names) else "",
              int(class_flags[i]) & flag_mask if i < len(class_flags) else 0,
              max(-127, min(127, int(class_layers[i]))) if i < len(class_layers) else 0,
              max(0, min(255, int(class_lanes[i]))) if i < len(class_lanes) else 0)
             for i, line in enumerate(roads[road_class])]
    pairs = [pair for pair in pairs if len(pair[0]) >= 2]
    if not pairs:
      continue
    lines = [pair[0] for pair in pairs]
    lengths = np.array([min(len(line), 65535) for line in lines], dtype=np.uint16)
    coords = np.concatenate([line[:65535] for line in lines])
    deltas = coords.copy()
    deltas[1:] -= coords[:-1]
    sections += _CLASS_HEADER.pack(road_class, len(lines), len(coords))
    sections += lengths.tobytes()
    if version >= 2:
      index = np.zeros(len(lines), dtype="<u2")
      for i, (_, name, *_) in enumerate(pairs):
        key = _clip_name(name or "")
        if key and (key in table or len(table) < 65535):
          index[i] = table.setdefault(key, len(table) + 1)
      sections += index.tobytes()
    if version >= 3:
      sections += np.array([pair[2] for pair in pairs], dtype=np.uint8).tobytes()
    if version >= 5:
      sections += np.array([pair[3] for pair in pairs], dtype=np.int8).tobytes()
      sections += np.array([pair[4] for pair in pairs], dtype=np.uint8).tobytes()
    sections += np.clip(deltas, -65535, 65535).astype(np.int32).astype("<i4").tobytes()
    classes += 1
  body = bytearray()
  if version >= 2:
    body += struct.pack("<H", len(table))
    for key in table:
      body += struct.pack("<B", len(key)) + key
  body += sections
  if version >= 4:
    kept = [(x, y, int(k)) for x, y, k in points if 0 < int(k) < 256 and (version >= 6 or int(k) in (SIGNAL, STOP))][:MAX_POINTS]
    body += struct.pack("<H", len(kept))
    for x, y, k in kept:
      body += struct.pack("<hhB", int(np.clip(round(x), -_COORD_LIMIT, _COORD_LIMIT)),
                          int(np.clip(round(y), -_COORD_LIMIT, _COORD_LIMIT)), k)
  if version >= 5:
    body += _kind_lines(walks, MAX_WALK_LINES, MAX_WALK_POINTS)
  if version >= 6:
    body += _kind_lines(rails, MAX_RAIL_LINES, MAX_RAIL_POINTS)
  return _HEADER.pack(MAGIC, version, classes) + zlib.compress(bytes(body), 6)


def _kind_lines(lines: Iterable[tuple[np.ndarray, int]], max_lines: int, max_points: int) -> bytes:
  """A table of lines with a kind byte each (the walks and the rails): int32 line count and point count,
  the uint16 lengths, the kind bytes, then the int32 x, y deltas."""
  kept, total = [], 0
  for line, kind in lines:
    line = np.clip(np.rint(np.asarray(line, dtype=np.float64)), -_COORD_LIMIT, _COORD_LIMIT).astype(np.int32)[:65535]
    if len(line) < 2 or not 0 < int(kind) < 256:
      continue
    if len(kept) >= max_lines or total + len(line) > max_points:
      break
    kept.append((line, int(kind)))
    total += len(line)
  out = bytearray(_WALK_HEADER.pack(len(kept), total))
  if kept:
    coords = np.concatenate([line for line, _ in kept])
    deltas = coords.copy()
    deltas[1:] -= coords[:-1]
    out += np.array([len(line) for line, _ in kept], dtype="<u2").tobytes()
    out += np.array([kind for _, kind in kept], dtype=np.uint8).tobytes()
    out += np.clip(deltas, -65535, 65535).astype("<i4").tobytes()
  return bytes(out)


def _decode_kind_lines(body: bytes, pos: int, max_lines: int, max_points: int,
                       what: str) -> tuple[tuple[np.ndarray, np.ndarray], np.ndarray, int]:
  """The table _kind_lines writes, at ``pos``: ((points, offsets), kinds, new pos)."""
  if pos + _WALK_HEADER.size > len(body):
    raise TileFormatError(f"truncated road tile {what}")
  count, total = _WALK_HEADER.unpack_from(body, pos)
  pos += _WALK_HEADER.size
  if count > max_lines or total > max_points or pos + count * 3 + total * 8 > len(body):
    raise TileFormatError(f"bad road tile {what}")
  lengths = np.frombuffer(body, dtype="<u2", count=count, offset=pos).astype(np.int32)
  kinds = np.frombuffer(body, dtype=np.uint8, count=count, offset=pos + count * 2).copy()
  pos += count * 3
  deltas = np.frombuffer(body, dtype="<i4", count=total * 2, offset=pos).reshape(total, 2)
  pos += total * 8
  if int(lengths.sum()) != total or (count and int(lengths.min()) < 2):
    raise TileFormatError(f"bad road tile {what} lengths")
  offsets = np.zeros(count + 1, dtype=np.int32)
  np.cumsum(lengths, out=offsets[1:])
  return (np.cumsum(deltas, axis=0).astype(np.float32), offsets), kinds, pos


@dataclass(frozen=True)
class RoadTile:
  key: TileKey
  # class -> (points float32 (N, 2) in tile units, line start offsets int32 (L + 1,))
  lines: dict[int, tuple[np.ndarray, np.ndarray]]
  # Distinct street names in the tile, and per class an index per line into them (0 = no name,
  # k = names[k - 1]). Empty for version 1 tiles.
  names: tuple[str, ...] = ()
  line_names: dict[int, np.ndarray] = dataclasses.field(default_factory=dict)
  version: int = VERSION
  # per class a flags byte per line (BRIDGE, TUNNEL); empty for version 1 and 2 tiles
  line_flags: dict[int, np.ndarray] = dataclasses.field(default_factory=dict)
  # (x, y, kind) points in tile units (SIGNAL, STOP); empty before version 4
  points: tuple[tuple[int, int, int], ...] = ()
  # per class a layer (int8) and lane count (uint8, 0 unknown) per line; empty before version 5
  line_layers: dict[int, np.ndarray] = dataclasses.field(default_factory=dict)
  line_lanes: dict[int, np.ndarray] = dataclasses.field(default_factory=dict)
  # walk lines (points float32 (N, 2), start offsets (W + 1,)) and a kind per line; None before version 5
  walks: tuple[np.ndarray, np.ndarray] | None = None
  walk_kinds: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(0, dtype=np.uint8))
  # rail lines in the walks' layout and a kind per line (MAJOR_RAIL.., maybe | RAIL_BRIDGE); None before version 6
  rails: tuple[np.ndarray, np.ndarray] | None = None
  rail_kinds: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(0, dtype=np.uint8))

  def name_of(self, road_class: int, line: int) -> str:
    index = self.line_names.get(road_class)
    if index is None or line >= len(index) or not index[line]:
      return ""
    return self.names[int(index[line]) - 1]

  def flags_of(self, road_class: int, line: int) -> int:
    flags = self.line_flags.get(road_class)
    return 0 if flags is None or line >= len(flags) else int(flags[line])

  def layer_of(self, road_class: int, line: int) -> int:
    layer = self.line_layers.get(road_class)
    return 0 if layer is None or line >= len(layer) else int(layer[line])

  def lanes_of(self, road_class: int, line: int) -> int:
    lanes = self.line_lanes.get(road_class)
    return 0 if lanes is None or line >= len(lanes) else int(lanes[line])

  def walk_lines(self) -> list[tuple[np.ndarray, int]]:
    if self.walks is None:
      return []
    points, offsets = self.walks
    return [(points[offsets[i]:offsets[i + 1]], int(self.walk_kinds[i])) for i in range(len(offsets) - 1)]

  def rail_lines(self) -> list[tuple[np.ndarray, int]]:
    if self.rails is None:
      return []
    points, offsets = self.rails
    return [(points[offsets[i]:offsets[i + 1]], int(self.rail_kinds[i])) for i in range(len(offsets) - 1)]

  @property
  def empty(self) -> bool:
    return not self.lines


def decode_road_tile(key: TileKey, raw: bytes) -> RoadTile:
  if len(raw) < _HEADER.size or len(raw) > MAX_STORED_BYTES:
    raise TileFormatError("bad road tile size")
  magic, version, classes = _HEADER.unpack_from(raw)
  if magic != MAGIC or not OLDEST_VERSION <= version <= VERSION or classes > len(CLASS_NAMES):
    raise TileFormatError("not a road tile")
  inflater = zlib.decompressobj()
  try:
    body = inflater.decompress(raw[_HEADER.size:], 32 * MAX_STORED_BYTES)
  except zlib.error as error:
    raise TileFormatError("corrupt road tile") from error
  if not inflater.eof or inflater.unused_data or inflater.unconsumed_tail:
    raise TileFormatError("truncated, oversized or padded road tile")
  pos, lines, names, line_names, line_flags, line_layers, line_lanes = 0, {}, [], {}, {}, {}, {}
  if version >= 2:
    if pos + 2 > len(body):
      raise TileFormatError("truncated road tile names")
    (count,), pos = struct.unpack_from("<H", body, pos), pos + 2
    for _ in range(count):
      if pos >= len(body) or pos + 1 + body[pos] > len(body):
        raise TileFormatError("truncated road tile names")
      size = body[pos]
      names.append(body[pos + 1:pos + 1 + size].decode("utf-8", "replace"))
      pos += 1 + size
  for _ in range(classes):
    if pos + _CLASS_HEADER.size > len(body):
      raise TileFormatError("truncated road tile")
    road_class, count, total = _CLASS_HEADER.unpack_from(body, pos)
    pos += _CLASS_HEADER.size
    need = count * (7 if version >= 5 else 5 if version >= 3 else 4 if version >= 2 else 2) + total * 8
    if road_class >= len(CLASS_NAMES) or road_class in lines or pos + need > len(body):
      raise TileFormatError("bad road tile class")
    lengths = np.frombuffer(body, dtype="<u2", count=count, offset=pos).astype(np.int32)
    pos += count * 2
    if version >= 2:
      index = np.frombuffer(body, dtype="<u2", count=count, offset=pos).astype(np.int32)
      pos += count * 2
      if count and int(index.max()) > len(names):
        raise TileFormatError("bad road tile name index")
      line_names[road_class] = index
    if version >= 3:
      line_flags[road_class] = np.frombuffer(body, dtype=np.uint8, count=count, offset=pos).copy()
      pos += count
    if version >= 5:
      line_layers[road_class] = np.frombuffer(body, dtype=np.int8, count=count, offset=pos).copy()
      line_lanes[road_class] = np.frombuffer(body, dtype=np.uint8, count=count, offset=pos + count).copy()
      pos += 2 * count
    deltas = np.frombuffer(body, dtype="<i4", count=total * 2, offset=pos).reshape(total, 2)
    pos += total * 8
    if int(lengths.sum()) != total or (count and int(lengths.min()) < 2):
      raise TileFormatError("bad road tile lengths")
    offsets = np.zeros(count + 1, dtype=np.int32)
    np.cumsum(lengths, out=offsets[1:])
    lines[road_class] = (np.cumsum(deltas, axis=0).astype(np.float32), offsets)
  points = []
  if version >= 4:
    if pos + 2 > len(body):
      raise TileFormatError("truncated road tile points")
    (count,), pos = struct.unpack_from("<H", body, pos), pos + 2
    if count > MAX_POINTS or pos + count * 5 > len(body):
      raise TileFormatError("bad road tile points")
    for _ in range(count):
      points.append(struct.unpack_from("<hhB", body, pos))
      pos += 5
  walks, walk_kinds = None, np.zeros(0, dtype=np.uint8)
  if version >= 5:
    walks, walk_kinds, pos = _decode_kind_lines(body, pos, MAX_WALK_LINES, MAX_WALK_POINTS, "walk")
  rails, rail_kinds = None, np.zeros(0, dtype=np.uint8)
  if version >= 6:
    rails, rail_kinds, pos = _decode_kind_lines(body, pos, MAX_RAIL_LINES, MAX_RAIL_POINTS, "rail")
  if pos != len(body):
    raise TileFormatError("trailing road tile data")
  return RoadTile(key, lines, tuple(names), line_names, version, line_flags, tuple(points),
                  line_layers, line_lanes, walks, walk_kinds, rails, rail_kinds)


def road_tile_version(raw: bytes) -> int | None:
  """Format version of a stored tile, or None when it is not one."""
  if len(raw) < _HEADER.size or raw[:4] != MAGIC:
    return None
  return raw[4]


def downgrade_road_tile(raw: bytes, version: int = OLDEST_VERSION) -> bytes:
  """The same tile in an older format, for a head unit that rejects newer ones.

  The head unit app before 2026-10-30 refuses any version but 1, and the one before 2026-10-31 any
  but 1 and 2; sending a newer tile would make it drop the tile and the comma resend it every few
  seconds forever. Going to 2 keeps the street names; only the flags are lost. Going to 3 keeps the
  flags; only the signal points are lost. Going to 4 loses the layers, lane counts, ONEWAY bit and walks.
  Going to 5 loses the rails and the level crossing points."""
  have = road_tile_version(raw)
  if have is None or have <= version:
    return raw
  tile = decode_road_tile(TileKey(DATA_ZOOM, 0, 0), raw)
  roads = {c: [points[offsets[i]:offsets[i + 1]] for i in range(len(offsets) - 1)] for c, (points, offsets) in tile.lines.items()}
  names = {c: [tile.name_of(c, i) for i in range(len(offsets) - 1)] for c, (_, offsets) in tile.lines.items()}
  flags = {c: [tile.flags_of(c, i) for i in range(len(offsets) - 1)] for c, (_, offsets) in tile.lines.items()}
  layers = {c: [tile.layer_of(c, i) for i in range(len(offsets) - 1)] for c, (_, offsets) in tile.lines.items()}
  lanes = {c: [tile.lanes_of(c, i) for i in range(len(offsets) - 1)] for c, (_, offsets) in tile.lines.items()}
  return encode_road_tile(roads, names if version >= 2 else None, version=version,
                          flags=flags if version >= 3 else None, points=tile.points,
                          layers=layers, lanes=lanes, walks=tile.walk_lines(), rails=tile.rail_lines())
