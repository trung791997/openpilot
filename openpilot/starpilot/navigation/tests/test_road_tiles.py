import gzip
import math
import zlib

import numpy as np
import pytest

from openpilot.starpilot.navigation.road_tiles import (
  DATA_ZOOM, EXTENT, LINK, MAJOR, MAX_NAME_BYTES, MINOR, MOTORWAY, STREET, TileFormatError, TileKey, corridor_tiles,
  decode_mvt_named_roads, decode_mvt_roads, decode_road_tile, downgrade_road_tile, encode_road_tile, lat_lon,
  meters_per_tile, road_tile_version, tiles_in_radius, world_xy,
)


# A minimal Mapbox Vector Tile writer, enough to exercise the reader.
def _varint(value):
  out = bytearray()
  while True:
    byte = value & 0x7F
    value >>= 7
    out.append(byte | (0x80 if value else 0))
    if not value:
      return bytes(out)


def _field(number, wire, payload):
  if wire == 0:
    return _varint(number << 3) + _varint(payload)
  return _varint(number << 3 | 2) + _varint(len(payload)) + payload


def _zigzag(value):
  return (value << 1) ^ (value >> 31)


def _geometry(lines):
  commands, x, y = [], 0, 0
  for line in lines:
    commands.append(1 | 1 << 3)
    commands += [_zigzag(line[0][0] - x), _zigzag(line[0][1] - y)]
    x, y = line[0]
    commands.append(2 | (len(line) - 1) << 3)
    for px, py in line[1:]:
      commands += [_zigzag(px - x), _zigzag(py - y)]
      x, y = px, py
  return b"".join(_varint(value) for value in commands)


def mvt(features, layer="road", extent=EXTENT, extra_layer=True):
  """features: (class, kind, lines) or (class, kind, lines, {other tag: string value})."""
  keys, values = ["class"], []
  for feature in features:
    for key in (feature[3] if len(feature) > 3 else {}):
      if key not in keys:
        keys.append(key)

  def value_index(value):
    if value not in values:
      values.append(value)
    return values.index(value)

  body = _field(15, 0, 2) + _field(1, 2, layer.encode())
  for feature_tags in features:
    cls, kind, lines = feature_tags[:3]
    tags = {"class": cls, **(feature_tags[3] if len(feature_tags) > 3 else {})}
    packed = b"".join(_varint(keys.index(key)) + _varint(value_index(value)) for key, value in tags.items())
    feature = _field(2, 2, packed) + _field(3, 0, kind) + _field(4, 2, _geometry(lines))
    body += _field(2, 2, feature)
  for key in keys:
    body += _field(3, 2, key.encode())
  for value in values:
    body += _field(4, 2, _field(1, 2, value.encode()))
  body += _field(5, 0, extent)
  tile = b""
  if extra_layer:
    water = _field(1, 2, b"water") + _field(2, 2, _field(3, 0, 3) + _field(4, 2, _geometry([[(0, 0), (10, 0), (10, 10)]])))
    tile += _field(3, 2, water)
  return tile + _field(3, 2, body)


def test_decoder_keeps_drawable_road_classes_only():
  data = mvt([("motorway", 2, [[(0, 0), (100, 50), (200, 50)]]),
              ("primary", 2, [[(10, 10), (20, 20)], [(30, 30), (40, 41)]]),
              ("street", 2, [[(-64, 5), (4160, 5)]]),
              ("motorway_link", 2, [[(1, 1), (2, 2)]]),
              ("service", 2, [[(5, 5), (6, 6)]]),
              ("path", 2, [[(7, 7), (8, 8)]]),
              ("major_rail", 2, [[(9, 9), (10, 10)]]),
              ("traffic_signals", 1, [[(1, 1), (1, 1)]])])
  roads = decode_mvt_roads(data)
  assert set(roads) == {MOTORWAY, MAJOR, STREET, LINK, MINOR}
  assert roads[MOTORWAY][0].tolist() == [[0, 0], [100, 50], [200, 50]]
  assert len(roads[MAJOR]) == 2 and roads[MAJOR][1].tolist() == [[30, 30], [40, 41]]
  assert roads[STREET][0].tolist() == [[-64, 5], [4160, 5]], "buffered coordinates past the tile edge survive"


def test_decoder_accepts_gzip_and_scales_other_extents():
  roads = decode_mvt_roads(gzip.compress(mvt([("trunk", 2, [[(0, 0), (512, 256)]])], extent=512)))
  assert roads[MOTORWAY][0].tolist() == [[0, 0], [4096, 2048]]


def test_decoder_rejects_truncated_tiles():
  data = mvt([("primary", 2, [[(0, 0), (100, 100)]])])
  with pytest.raises(TileFormatError):
    decode_mvt_roads(data[:-7])


def test_compact_tile_round_trip_is_smaller_and_exact():
  rng = np.random.default_rng(3)
  roads = {cls: [np.cumsum(rng.integers(-40, 40, size=(rng.integers(2, 30), 2)), axis=0) + 2048 for _ in range(60)]
           for cls in (MOTORWAY, STREET, MINOR)}
  raw = encode_road_tile(roads)
  tile = decode_road_tile(TileKey(14, 1, 2), raw)
  assert set(tile.lines) == set(roads)
  for cls, lines in roads.items():
    points, offsets = tile.lines[cls]
    assert len(offsets) == len(lines) + 1
    for index, line in enumerate(lines):
      assert points[offsets[index]:offsets[index + 1]].tolist() == line.tolist()
  assert len(raw) < sum(line.size * 4 for lines in roads.values() for line in lines)


def test_empty_tile_and_corruption():
  key = TileKey(14, 0, 0)
  assert decode_road_tile(key, encode_road_tile({})).empty
  raw = encode_road_tile({MAJOR: [np.array([[0, 0], [5, 5]])]})
  for bad in (b"", b"XXXX" + raw[4:], raw[:-3], raw + b"\0"):
    with pytest.raises(TileFormatError):
      decode_road_tile(key, bad)


def test_decoder_reads_street_names_with_ref_as_fallback():
  data = mvt([("primary", 2, [[(0, 0), (10, 0)], [(10, 0), (20, 0)]], {"name": "Main Street"}),
              ("motorway", 2, [[(0, 5), (50, 5)]], {"ref": "I 5"}),
              ("motorway", 2, [[(0, 9), (50, 9)]], {"name": "Golden State Freeway", "ref": "I 5"}),
              ("street", 2, [[(1, 1), (2, 2)]])])
  roads, names = decode_mvt_named_roads(data)
  assert names[MAJOR] == ["Main Street", "Main Street"], "one name per line, even when a feature has several"
  assert names[MOTORWAY] == ["I 5", "Golden State Freeway"]
  assert names[STREET] == [""]
  assert all(len(names[cls]) == len(roads[cls]) for cls in roads)
  assert decode_mvt_roads(data).keys() == roads.keys()


def test_names_are_stored_once_per_tile_and_survive_the_round_trip():
  line = np.array([[0, 0], [100, 0]])
  roads = {MAJOR: [line, line + 1, line + 2], STREET: [line + 3, line + 4]}
  names = {MAJOR: ["Main Street", "Main Street", ""], STREET: ["Main Street", "Ünïcode Ave"]}
  raw = encode_road_tile(roads, names)
  assert road_tile_version(raw) == 2
  tile = decode_road_tile(TileKey(14, 0, 0), raw)
  assert tile.names == ("Main Street", "Ünïcode Ave"), "a street split into many lines is stored once"
  assert [tile.name_of(MAJOR, i) for i in range(3)] == ["Main Street", "Main Street", ""]
  assert [tile.name_of(STREET, i) for i in range(2)] == ["Main Street", "Ünïcode Ave"]
  assert tile.name_of(MINOR, 0) == ""
  # the names table costs little next to the same tile without names
  assert len(raw) - len(encode_road_tile(roads)) < 40
  # an over-long name is cut at a character boundary, never mid-character
  long = encode_road_tile({MAJOR: [line]}, {MAJOR: ["é" * 100]})
  assert decode_road_tile(TileKey(14, 0, 0), long).names == ("é" * (MAX_NAME_BYTES // 2),)


def test_version_1_tiles_still_decode_and_new_ones_downgrade_for_old_head_units():
  line = np.array([[0, 0], [100, 50], [90, 70]])
  roads, names = {MOTORWAY: [line], STREET: [line + 5]}, {MOTORWAY: ["I 5"], STREET: ["Elm"]}
  old = encode_road_tile(roads, version=1)
  assert road_tile_version(old) == 1 and old[4] == 1
  tile = decode_road_tile(TileKey(14, 0, 0), old)
  assert tile.version == 1 and tile.names == () and tile.name_of(MOTORWAY, 0) == ""
  new = encode_road_tile(roads, names)
  down = downgrade_road_tile(new)
  assert down == old, "the head unit before 2026-10-30 gets exactly the old format"
  assert downgrade_road_tile(old) == old and downgrade_road_tile(b"fake") == b"fake"
  # an index past the names table is corrupt, not a crash later in the renderer
  bad = bytearray(new)
  body = bytearray(zlib.decompress(bytes(bad[6:])))
  body[2 + 1 + 3 + 1 + 3 + 9 + 2] = 9  # first motorway line's name index
  with pytest.raises(TileFormatError):
    decode_road_tile(TileKey(14, 0, 0), bytes(bad[:6]) + zlib.compress(bytes(body)))


def test_mercator_round_trip_and_scale():
  x, y = world_xy(36.1147, -115.1728)
  assert int(x) == 2950 and int(y) == 6427  # z14 tile 2950/6427 covers the Las Vegas Strip
  latitude, longitude = lat_lon(x, y)
  assert math.isclose(latitude, 36.1147, abs_tol=1e-9) and math.isclose(longitude, -115.1728, abs_tol=1e-9)
  assert 1950 < meters_per_tile(36.1) < 2000


def test_radius_and_corridor_tiles():
  keys = tiles_in_radius(36.1147, -115.1728, 5000)
  center = TileKey(DATA_ZOOM, *map(int, world_xy(36.1147, -115.1728)))
  assert keys[0] == center and len(set(keys)) == len(keys)
  assert 15 <= len(keys) <= 40
  route = [(36.10, -115.17), (36.20, -115.17)]
  corridor = corridor_tiles(route, radius_tiles=0)
  assert corridor[0] == TileKey(DATA_ZOOM, *map(int, world_xy(*route[0])))
  assert corridor[-1] == TileKey(DATA_ZOOM, *map(int, world_xy(*route[-1])))
  assert len(corridor) == len(set(corridor)) >= 5
