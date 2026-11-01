import gzip
import math
import zlib

import numpy as np
import pytest

from openpilot.starpilot.navigation.road_tiles import (
  CROSSING, FOOTWAY, ONEWAY, PEDESTRIAN, SIDEWALK, decode_mvt_road_detail,
  LEVEL_CROSSING, MAJOR_RAIL, MAX_RAIL_LINES, MINOR_RAIL, RAIL_BRIDGE, SERVICE_RAIL,
  BRIDGE, DATA_ZOOM, EXTENT, LINK, MAJOR, MAX_NAME_BYTES, MINOR, MOTORWAY, SIGNAL, STOP, STREET, TUNNEL, VERSION, TileFormatError,
  TileKey, corridor_tiles, decode_mvt_named_roads, decode_mvt_road_layer, decode_mvt_road_tile, decode_mvt_roads, decode_road_tile,
  downgrade_road_tile, encode_road_tile, lat_lon,
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


def _point_geometry(groups):
  """One MoveTo per group of points (a multipoint feature when a group has more than one)."""
  commands, x, y = [], 0, 0
  for group in groups:
    commands.append(1 | len(group) << 3)
    for px, py in group:
      commands += [_zigzag(px - x), _zigzag(py - y)]
      x, y = px, py
  return b"".join(_varint(value) for value in commands)


def mvt(features, layer="road", extent=EXTENT, extra_layer=True):
  """features: (class, kind, lines) or (class, kind, lines, {other tag: value}); a value is a string, a bool
  (written as a bool) or an int (written as a sint); kind 1 (point) takes groups of points instead of lines."""
  keys, values = ["class"], []
  for feature in features:
    for key in (feature[3] if len(feature) > 3 else {}):
      if key not in keys:
        keys.append(key)

  def value_index(value):
    typed = (type(value).__name__, value)
    if typed not in values:
      values.append(typed)
    return values.index(typed)

  body = _field(15, 0, 2) + _field(1, 2, layer.encode())
  for feature_tags in features:
    cls, kind, lines = feature_tags[:3]
    tags = {"class": cls, **(feature_tags[3] if len(feature_tags) > 3 else {})}
    packed = b"".join(_varint(keys.index(key)) + _varint(value_index(value)) for key, value in tags.items())
    geometry = _point_geometry(lines) if kind == 1 else _geometry(lines)
    feature = _field(2, 2, packed) + _field(3, 0, kind) + _field(4, 2, geometry)
    body += _field(2, 2, feature)
  for key in keys:
    body += _field(3, 2, key.encode())
  for kind, value in values:
    if kind == "bool":
      body += _field(4, 2, _field(7, 0, int(value)))
    elif kind == "int":
      body += _field(4, 2, _field(6, 0, (value << 1) ^ (value >> 63)))
    else:
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
  assert road_tile_version(raw) == VERSION == 6
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


def test_decoder_keeps_bridges_and_tunnels_from_the_structure_tag():
  data = mvt([("motorway", 2, [[(0, 0), (100, 0)]], {"name": "I-635", "structure": "bridge"}),
              ("motorway", 2, [[(0, 9), (100, 9)]], {"structure": "none"}),
              ("street", 2, [[(5, 5), (6, 50)], [(7, 7), (9, 9)]], {"structure": "tunnel"}),
              ("street", 2, [[(1, 1), (2, 2)]], {"structure": "ford"}),
              ("primary", 2, [[(1, 1), (2, 2)]])])
  roads, names, flags = decode_mvt_road_layer(data)
  assert flags == {MOTORWAY: [BRIDGE, 0], STREET: [TUNNEL, TUNNEL, 0], MAJOR: [0]}
  assert names[MOTORWAY] == ["I-635", ""]
  assert all(len(flags[cls]) == len(roads[cls]) for cls in roads)


def test_bridge_flags_survive_the_round_trip_and_older_versions_drop_only_what_they_lack():
  line = np.array([[0, 0], [100, 50], [90, 70]])
  roads = {MOTORWAY: [line, line + 1], STREET: [line + 5], LINK: [line + 9]}
  names = {MOTORWAY: ["I 5", ""], STREET: ["Elm"], LINK: [""]}
  flags = {MOTORWAY: [0, BRIDGE], STREET: [TUNNEL], LINK: [BRIDGE]}
  raw = encode_road_tile(roads, names, flags=flags)
  tile = decode_road_tile(TileKey(14, 0, 0), raw)
  assert tile.version == VERSION
  assert [tile.flags_of(MOTORWAY, i) for i in range(2)] == [0, BRIDGE]
  assert tile.flags_of(STREET, 0) == TUNNEL and tile.flags_of(LINK, 0) == BRIDGE and tile.flags_of(MINOR, 0) == 0
  assert tile.name_of(STREET, 0) == "Elm"
  # a byte a line, and the flags compress to almost nothing next to the same tile without them
  assert len(raw) - len(encode_road_tile(roads, names, version=3)) < 8
  assert len(raw) - len(encode_road_tile(roads, names, version=2)) < 16
  # the head unit app of 2026-10-30 reads names but not flags: it gets exactly the version 2 tile
  v2 = downgrade_road_tile(raw, 2)
  assert v2 == encode_road_tile(roads, names, version=2)
  old = decode_road_tile(TileKey(14, 0, 0), v2)
  assert old.version == 2 and old.line_flags == {} and old.flags_of(MOTORWAY, 1) == 0 and old.name_of(STREET, 0) == "Elm"
  assert downgrade_road_tile(raw, 1) == encode_road_tile(roads, version=1)
  assert downgrade_road_tile(raw, 3) == encode_road_tile(roads, names, version=3, flags=flags), "the flags survive"
  assert downgrade_road_tile(raw, 6) == raw and downgrade_road_tile(v2, 3) == v2, "never upgraded"
  # the same lines in every version
  for version in (1, 2, 3):
    decoded = decode_road_tile(TileKey(14, 0, 0), downgrade_road_tile(raw, version))
    for cls, (points, offsets) in tile.lines.items():
      assert np.array_equal(decoded.lines[cls][0], points) and np.array_equal(decoded.lines[cls][1], offsets)


def test_a_cut_flags_array_is_corrupt_not_misread():
  line = np.array([[0, 0], [100, 0]])
  raw = encode_road_tile({MAJOR: [line]}, {MAJOR: ["Main"]}, flags={MAJOR: [BRIDGE]})
  body = zlib.decompress(raw[6:])
  for cut in (1, 9):
    with pytest.raises(TileFormatError):
      decode_road_tile(TileKey(14, 0, 0), raw[:6] + zlib.compress(body[:-cut]))
  with pytest.raises(TileFormatError):
    decode_road_tile(TileKey(14, 0, 0), b"SPRT\x07\x01" + zlib.compress(body))


def test_decoder_keeps_traffic_signal_points_inside_the_tile_only():
  data = mvt([("primary", 2, [[(0, 0), (4096, 4096)]], {"name": "Main St"}),
              ("traffic_signals", 1, [[(100, 200)], [(4000, 50), (4010, 60)]]),
              ("traffic_signals", 1, [[(-20, 300)]]),       # in the buffer: the neighbouring tile has it
              ("traffic_signals", 1, [[(4096, 300)]]),
              ("turning_circle", 1, [[(500, 500)]]),
              ("junction", 1, [[(600, 600)]])])
  roads, names, flags, points = decode_mvt_road_tile(data)
  assert points == [(100, 200, SIGNAL), (4000, 50, SIGNAL), (4010, 60, SIGNAL)]
  assert names[MAJOR] == ["Main St"] and flags[MAJOR] == [0] and len(roads[MAJOR]) == 1
  assert decode_mvt_road_layer(data)[0].keys() == roads.keys(), "the three-list reader is unchanged"
  half = decode_mvt_road_tile(mvt([("traffic_signals", 1, [[(256, 100)]])], extent=2048))[3]
  assert half == [(512, 200, SIGNAL)], "other extents scale like the lines"


def test_signal_points_round_trip_and_older_versions_drop_only_the_points():
  line = np.array([[0, 0], [100, 50]])
  roads, names, flags = {MAJOR: [line]}, {MAJOR: ["Main"]}, {MAJOR: [BRIDGE]}
  points = [(100.4, 200.6, SIGNAL), (4095, 0, SIGNAL), (7, 8, STOP), (1, 1, 0)]
  raw = encode_road_tile(roads, names, flags=flags, points=points)
  tile = decode_road_tile(TileKey(14, 0, 0), raw)
  assert tile.version == 6 == VERSION
  assert tile.points == ((100, 201, SIGNAL), (4095, 0, SIGNAL), (7, 8, STOP)), "rounded; kind 0 is not a point"
  assert tile.flags_of(MAJOR, 0) == BRIDGE and tile.name_of(MAJOR, 0) == "Main"
  # five bytes a point, and nothing when there are none
  assert len(zlib.decompress(raw[6:])) - len(zlib.decompress(encode_road_tile(roads, names, flags=flags)[6:])) == 15
  assert decode_road_tile(TileKey(14, 0, 0), encode_road_tile(roads, names, flags=flags)).points == ()
  # the head unit app before signals reads version 3: it gets exactly the version 3 tile, flags kept
  v3 = downgrade_road_tile(raw, 3)
  assert v3 == encode_road_tile(roads, names, version=3, flags=flags)
  old = decode_road_tile(TileKey(14, 0, 0), v3)
  assert old.version == 3 and old.points == () and old.flags_of(MAJOR, 0) == BRIDGE
  assert downgrade_road_tile(raw, 2) == encode_road_tile(roads, names, version=2)
  assert downgrade_road_tile(raw, 1) == encode_road_tile(roads, version=1)
  for version in (1, 2, 3):
    assert decode_road_tile(TileKey(14, 0, 0), downgrade_road_tile(raw, version)).points == ()


def test_a_cut_or_padded_points_table_is_corrupt_not_misread():
  raw = encode_road_tile({MAJOR: [np.array([[0, 0], [100, 0]])]}, points=[(5, 5, SIGNAL), (9, 9, SIGNAL)])
  body = zlib.decompress(raw[6:])
  for bad in (body[:-1], body[:-5], body[:-11], body + b"\0"):
    with pytest.raises(TileFormatError):
      decode_road_tile(TileKey(14, 0, 0), raw[:6] + zlib.compress(bad))
  # a version 3 tile carries no points table, so the same bytes read as version 3 are trailing data
  with pytest.raises(TileFormatError):
    decode_road_tile(TileKey(14, 0, 0), b"SPRT\x03" + raw[5:6] + zlib.compress(body))


def test_decoder_reads_layer_lanes_oneway_and_keeps_sidewalks_crossings_and_footways_as_walks():
  data = mvt([("motorway", 2, [[(0, 0), (4000, 0)]], {"structure": "bridge", "layer": 2, "oneway": True, "lane_count": 4}),
              ("motorway_link", 2, [[(0, 100), (400, 100)]], {"layer": -1, "oneway": "true"}),
              ("primary", 2, [[(0, 200), (4000, 200)]], {"layer": 99, "oneway": False}),
              ("path", 2, [[(10, 10), (20, 10)]], {"type": "sidewalk"}),
              ("path", 2, [[(30, 10), (30, 40)]], {"type": "crossing"}),
              ("path", 2, [[(50, 10), (60, 10)]], {"type": "footway"}),
              ("path", 2, [[(70, 10), (80, 10)]], {"type": "cycleway"}),
              ("path", 2, [[(90, 10), (99, 10)]], {"type": "steps"}),
              ("pedestrian", 2, [[(100, 10), (200, 10)]])])
  detail = decode_mvt_road_detail(data)
  assert detail.flags[MOTORWAY] == [BRIDGE | ONEWAY] and detail.layers[MOTORWAY] == [2] and detail.lanes[MOTORWAY] == [4]
  assert detail.flags[LINK] == [ONEWAY] and detail.layers[LINK] == [-1] and detail.lanes[LINK] == [0]
  assert detail.flags[MAJOR] == [0] and detail.layers[MAJOR] == [15], "an absurd layer is clipped"
  assert detail.walk_kinds == [SIDEWALK, CROSSING, FOOTWAY, PEDESTRIAN], "cycleways and steps are not drawn"
  assert detail.walks[1].tolist() == [[30, 10], [30, 40]]
  roads, names, flags, points = decode_mvt_road_tile(data)
  assert roads.keys() == detail.roads.keys() and flags == detail.flags, "the version 4 reader is the same content"
  tile = decode_road_tile(TileKey(14, 0, 0), detail.encode())
  assert tile.version == VERSION and tile.flags_of(MOTORWAY, 0) == BRIDGE | ONEWAY
  assert tile.layer_of(MOTORWAY, 0) == 2 and tile.lanes_of(MOTORWAY, 0) == 4 and tile.layer_of(LINK, 0) == -1
  assert [(w.tolist(), k) for w, k in tile.walk_lines()] == [(w.tolist(), k) for w, k in zip(detail.walks, detail.walk_kinds, strict=True)]


def test_version_5_downgrades_to_exactly_the_version_4_tile_and_old_tiles_have_no_walks():
  line = np.array([[0, 0], [100, 50]])
  roads, names, flags = {MOTORWAY: [line, line + 5]}, {MOTORWAY: ["I-405", ""]}, {MOTORWAY: [BRIDGE | ONEWAY, ONEWAY]}
  points = [(5, 5, SIGNAL)]
  walks = [(np.array([[0, 10], [40, 10], [40, 30]]), SIDEWALK), (np.array([[1, 1], [1, 9]]), CROSSING)]
  raw = encode_road_tile(roads, names, flags=flags, points=points, layers={MOTORWAY: [1, 0]}, lanes={MOTORWAY: [5, 0]},
                         walks=walks)
  v4 = downgrade_road_tile(raw, 4)
  assert v4 == encode_road_tile(roads, names, version=4, flags={MOTORWAY: [BRIDGE, 0]}, points=points)
  old = decode_road_tile(TileKey(14, 0, 0), v4)
  assert old.flags_of(MOTORWAY, 0) == BRIDGE, "the old app sees no new bit"
  assert old.walk_lines() == [] and old.layer_of(MOTORWAY, 0) == 0 and old.lanes_of(MOTORWAY, 0) == 0
  assert downgrade_road_tile(raw, 3) == encode_road_tile(roads, names, version=3, flags={MOTORWAY: [BRIDGE, 0]})
  tile = decode_road_tile(TileKey(14, 0, 0), raw)
  assert [tile.layer_of(MOTORWAY, i) for i in range(2)] == [1, 0] and tile.lanes_of(MOTORWAY, 0) == 5
  assert [k for _, k in tile.walk_lines()] == [SIDEWALK, CROSSING]
  assert tile.walk_lines()[0][0].tolist() == [[0, 10], [40, 10], [40, 30]]
  assert downgrade_road_tile(raw, 6) == raw
  # no walks or rails: 8 bytes of table header each; two bytes a line for layer and lanes
  bare = encode_road_tile(roads, names, flags=flags, points=points)
  v4_bare = encode_road_tile(roads, names, version=4, flags=flags, points=points)
  assert len(zlib.decompress(bare[6:])) - len(zlib.decompress(v4_bare[6:])) == 8 + 8 + 2 * 2


def test_a_cut_or_padded_walk_table_is_corrupt_not_misread():
  raw = encode_road_tile({MAJOR: [np.array([[0, 0], [100, 0]])]}, walks=[(np.array([[0, 0], [9, 9]]), SIDEWALK)])
  body = zlib.decompress(raw[6:])
  for bad in (body[:-1], body[:-8], body[:-19], body + b"\0"):
    with pytest.raises(TileFormatError):
      decode_road_tile(TileKey(14, 0, 0), raw[:6] + zlib.compress(bad))
  short = encode_road_tile({MAJOR: [np.array([[0, 0], [100, 0]])]}, walks=[(np.array([[0, 0]]), SIDEWALK),
                                                                           (np.array([[0, 0], [5, 5]]), 0)])
  assert decode_road_tile(TileKey(14, 0, 0), short).walk_lines() == [], "a one-point or kind-0 walk is not stored"


def test_decoder_keeps_level_crossings_and_rails_but_not_rail_tunnels():
  data = mvt([("primary", 2, [[(0, 2000), (4096, 2000)]]),
              ("level_crossing", 1, [[(1000, 2000)]]),
              ("traffic_signals", 1, [[(3000, 2000)]]),
              ("major_rail", 2, [[(1000, 0), (1000, 4096)]]),
              ("minor_rail", 2, [[(2000, 0), (2100, 4096)]], {"structure": "bridge"}),
              ("service_rail", 2, [[(500, 500), (600, 600), (700, 600)]], {"type": "siding"}),
              ("major_rail", 2, [[(3000, 0), (3000, 4096)]], {"structure": "tunnel"}),
              ("level_crossing", 1, [[(-5, 100)]])])
  detail = decode_mvt_road_detail(data)
  assert detail.points == [(1000, 2000, LEVEL_CROSSING), (3000, 2000, SIGNAL)], "the buffer crossing is the neighbour's"
  assert detail.rail_kinds == [MAJOR_RAIL, MINOR_RAIL | RAIL_BRIDGE, SERVICE_RAIL], "a subway in a tunnel is not drawn"
  assert list(detail.roads) == [MAJOR], "rails are not roads"
  tile = decode_road_tile(TileKey(14, 0, 0), detail.encode())
  assert tile.version == VERSION == 6
  assert tile.points == ((1000, 2000, LEVEL_CROSSING), (3000, 2000, SIGNAL))
  assert [(r.tolist(), k) for r, k in tile.rail_lines()] == [(r.tolist(), k) for r, k in zip(detail.rails, detail.rail_kinds, strict=True)]
  assert tile.walk_lines() == []


def test_version_6_downgrades_to_exactly_the_version_5_tile_without_rails_or_crossings():
  roads, names = {MAJOR: [np.array([[0, 0], [100, 50]])]}, {MAJOR: ["Bolsa Ave"]}
  points = [(5, 5, SIGNAL), (50, 25, LEVEL_CROSSING), (9, 9, STOP)]
  walks = [(np.array([[0, 10], [40, 10]]), SIDEWALK)]
  rails = [(np.array([[50, 0], [50, 100]]), MAJOR_RAIL), (np.array([[0, 0], [5, 5]]), SERVICE_RAIL | RAIL_BRIDGE)]
  raw = encode_road_tile(roads, names, points=points, walks=walks, rails=rails)
  v5 = downgrade_road_tile(raw, 5)
  assert v5 == encode_road_tile(roads, names, version=5, points=[(5, 5, SIGNAL), (9, 9, STOP)], walks=walks)
  old = decode_road_tile(TileKey(14, 0, 0), v5)
  assert old.rail_lines() == [] and old.rails is None and [k for *_, k in old.points] == [SIGNAL, STOP]
  assert [k for *_, k in decode_road_tile(TileKey(14, 0, 0), downgrade_road_tile(raw, 4)).points] == [SIGNAL, STOP]
  tile = decode_road_tile(TileKey(14, 0, 0), raw)
  assert [k for _, k in tile.rail_lines()] == [MAJOR_RAIL, SERVICE_RAIL | RAIL_BRIDGE]
  assert tile.rail_lines()[0][0].tolist() == [[50, 0], [50, 100]]
  assert [k for *_, k in tile.points] == [SIGNAL, LEVEL_CROSSING, STOP]


def test_a_cut_or_padded_rail_table_is_corrupt_not_misread_and_the_rails_are_capped():
  raw = encode_road_tile({MAJOR: [np.array([[0, 0], [100, 0]])]}, rails=[(np.array([[0, 0], [9, 9]]), MAJOR_RAIL)])
  body = zlib.decompress(raw[6:])
  for bad in (body[:-1], body[:-8], body + b"\0"):
    with pytest.raises(TileFormatError):
      decode_road_tile(TileKey(14, 0, 0), raw[:6] + zlib.compress(bad))
  many = [(np.array([[i, 0], [i, 9]]), MINOR_RAIL) for i in range(MAX_RAIL_LINES + 10)]
  tile = decode_road_tile(TileKey(14, 0, 0), encode_road_tile({MAJOR: [np.array([[0, 0], [100, 0]])]}, rails=many))
  assert len(tile.rail_lines()) == MAX_RAIL_LINES
