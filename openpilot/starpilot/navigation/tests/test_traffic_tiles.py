import gzip
import struct
import zlib

import numpy as np
import pytest

from openpilot.starpilot.navigation import traffic_tiles as tt
from openpilot.starpilot.navigation.road_tiles import TileFormatError, encode_road_tile, MOTORWAY
from openpilot.starpilot.navigation.tests.test_road_tiles import _field, _geometry, _varint


def traffic_mvt(features, extent=4096, layer="traffic"):
  """features: (congestion or None, [[(x, y), ...], ...]) as LineString/MultiLineString."""
  keys, values = ["congestion"], []
  body = _field(15, 0, 2) + _field(1, 2, layer.encode())
  for congestion, lines in features:
    tags = b""
    if congestion is not None:
      if congestion not in values:
        values.append(congestion)
      tags = _varint(0) + _varint(values.index(congestion))
    feature = (_field(2, 2, tags) if tags else b"") + _field(3, 0, 2) + _field(4, 2, _geometry(lines))
    body += _field(2, 2, feature)
  for key in keys:
    body += _field(3, 2, key.encode())
  for value in values:
    body += _field(4, 2, _field(1, 2, value.encode()))
  body += _field(5, 0, extent)
  return _field(3, 2, body)


def test_payload_round_trip_keeps_levels_and_points():
  lines = [(2, np.array([[0, 0], [100, 50], [4200, -30]])), (4, np.array([[5, 5], [6, 7]])), (3, np.array([[40000, 1], [2, 2]]))]
  raw = tt.encode_traffic_tile(lines, 1_700_000_000)
  assert raw[:4] == b"SPTQ" and raw[4] == 1 and raw[5] == 0
  assert struct.unpack_from("<I", raw, 6)[0] == 1_700_000_000
  fetched, back = tt.decode_traffic_payload(raw)
  assert fetched == 1_700_000_000
  assert [level for level, _ in back] == [2, 4, 3]
  assert back[0][1].tolist() == [[0, 0], [100, 50], [4200, -30]]
  assert back[2][1].tolist() == [[32767, 1], [2, 2]], "clipped like road tile coordinates"


def test_levels_below_moderate_and_one_point_lines_are_dropped():
  raw = tt.encode_traffic_tile([(1, np.array([[0, 0], [1, 1]])), (0, np.array([[0, 0], [1, 1]])),
                                (3, np.array([[0, 0]])), (3, np.array([[0, 0], [1, 1]]))], 5)
  assert [level for level, _ in tt.decode_traffic_payload(raw)[1]] == [3]


def test_empty_tile_is_line_count_zero():
  raw = tt.encode_traffic_tile([], 7)
  assert struct.unpack("<H", zlib.decompress(raw[10:]))[0] == 0
  assert tt.decode_traffic_payload(raw) == (7, [])
  assert tt.payload_of(b"", 7) == raw


def test_points_are_coded_as_road_tile_lines_are():
  line = np.array([[10, 20], [15, 18], [-3, 4100]])
  road = zlib.decompress(encode_road_tile({MOTORWAY: [line]}, version=1)[6:])
  deltas = np.frombuffer(road, dtype="<i4", offset=len(road) - 6 * 4)
  body = zlib.decompress(tt.encode_traffic_tile([(2, line)], 0)[10:])
  assert body[2 + 3:] == deltas.tobytes()


def test_mvt_decode_keeps_congested_lines_only_and_scales_extent():
  data = traffic_mvt([("low", [[(0, 0), (10, 10)]]), ("moderate", [[(0, 0), (256, 128)], [(1, 1), (2, 2)]]),
                      ("heavy", [[(5, 5), (6, 6)]]), ("severe", [[(7, 7), (8, 8)]]), (None, [[(1, 1), (2, 2)]]),
                      ("closed", [[(1, 1), (2, 2)]])], extent=512)
  lines = tt.decode_traffic_mvt(gzip.compress(data))
  assert [level for level, _ in lines] == [2, 2, 3, 4]
  assert lines[0][1].tolist() == [[0, 0], [2048, 1024]]


def test_mvt_other_layers_and_garbage():
  assert tt.decode_traffic_mvt(traffic_mvt([("severe", [[(0, 0), (9, 9)]])], layer="road")) == []
  with pytest.raises(TileFormatError):
    tt.decode_traffic_mvt(traffic_mvt([("severe", [[(0, 0), (9, 9)]])])[:-7])


class Rig:
  def __init__(self, token="tok", mvt=None):
    self.now, self.calls, self.fail, self.logs = [1000.], [], False, []
    self.mvt = mvt or traffic_mvt([("severe", [[(0, 0), (9, 9)]])])
    self.token = token

    def fetch(key, token):
      self.calls.append((key, self.now[0]))
      if self.fail:
        raise OSError("down")
      return self.mvt
    self.t = tt.TrafficTiles(fetch=fetch, token=lambda: self.token, clock=lambda: self.now[0], wall=lambda: 1_700_000_000.,
                             log=lambda *a, **k: self.logs.append((a, k)))
    self.t._thread = object()  # tests drive poll() themselves, no thread

  def want(self, keys):
    self.t.want(keys)

  def drain(self):
    while self.t.poll():
      pass


KEYS = [f"{x}/{y}" for x in range(3) for y in range(3)]


def test_no_wanted_tiles_no_fetch_and_idle_stops_fetching():
  r = Rig()
  assert not r.t.poll() and not r.calls
  r.want(KEYS[:2])
  r.t.idle()
  r.drain()
  assert not r.calls
  r.want(KEYS[:2])
  r.now[0] += tt.WANT_FRESH_S + 1  # hu_feed stopped asking
  r.drain()
  assert not r.calls


def test_no_token_no_fetch_and_one_log():
  r = Rig(token="")
  r.want(KEYS)
  r.drain()
  r.now[0] += 1
  r.want(KEYS)
  r.drain()
  assert not r.calls
  assert [k for a, k in r.logs if k.get("status") == "no_token"] == [{"status": "no_token"}]


def test_fetches_nearest_first_and_refreshes_every_300s():
  r = Rig()
  r.want(KEYS[:3])
  r.drain()
  assert [k for k, _ in r.calls] == KEYS[:3]
  r.now[0] += 299
  r.want(KEYS[:3])
  r.drain()
  assert len(r.calls) == 3
  r.now[0] += 2
  r.want(KEYS[:3])
  r.drain()
  assert len(r.calls) == 6


def test_at_most_nine_fetches_per_window():
  r = Rig()
  keys = KEYS + ["9/9", "9/8"]
  r.want(keys)
  r.drain()
  assert len(r.calls) == tt.MAX_FETCHES
  r.now[0] += 10
  r.want(keys)
  r.drain()
  assert len(r.calls) == tt.MAX_FETCHES, "the 10th waits for the window"
  r.now[0] += tt.REFRESH_S
  r.want(keys)
  r.drain()
  assert len(r.calls) > tt.MAX_FETCHES
  assert max(sum(1 for _, t in r.calls if t0 <= t < t0 + tt.REFRESH_S) for _, t0 in r.calls) <= tt.MAX_FETCHES


def test_errors_back_off_per_tile_doubling_to_the_cap():
  r = Rig()
  r.fail = True
  r.want(["1/1", "1/2"])
  r.drain()
  assert [k for k, _ in r.calls] == ["1/1", "1/2"]
  gaps, last = [], r.now[0]
  for _ in range(12):
    n = len(r.calls)
    while len(r.calls) == n:
      r.now[0] += 1
      r.want(["1/1"])
      r.t._attempts.clear()  # isolate the backoff from the 9 per window cap
      r.t.poll()
    gaps.append(r.now[0] - last)
    last = r.now[0]
  assert gaps[0] == 59 + 1 - 1 + 0 or gaps[0] in (59., 60.)
  assert gaps[1:4] == [120., 240., 480.]
  assert max(gaps) == tt.BACKOFF_MAX_S and gaps[-1] == tt.BACKOFF_MAX_S
  r.fail = False
  r.now[0] += tt.BACKOFF_MAX_S
  r.want(["1/1"])
  r.drain()
  assert r.t.current(["1/1"])


def test_payload_cached_and_generation_changes_only_when_content_changes():
  r = Rig()
  r.want(["1/1"])
  r.drain()
  (key, gen, payload), = r.t.current(["1/1", "5/5"])
  assert key == "1/1" and tt.decode_traffic_payload(payload)[0] == 1_700_000_000
  r.now[0] += 301
  r.want(["1/1"])
  r.drain()
  assert r.t.current(["1/1"])[0][1] == gen
  r.mvt = traffic_mvt([])
  r.now[0] += 301
  r.want(["1/1"])
  r.drain()
  (_, gen2, payload2), = r.t.current(["1/1"])
  assert gen2 > gen and tt.decode_traffic_payload(payload2)[1] == []


def test_counts_logged_at_most_once_a_minute():
  r = Rig()
  r.want(KEYS[:2])
  for _ in range(5):
    r.drain()
    r.now[0] += 10
    r.want(KEYS[:2])
  counts = [k for a, k in r.logs if a == ("hu_feed_traffic",) and "fetches" in k]
  assert len(counts) == 1 and counts[0]["fetches"] in (0, 2)
  r.now[0] += tt.LOG_EVERY_S
  r.want(KEYS[:2])
  r.drain()
  assert len([k for a, k in r.logs if "fetches" in k]) == 2
