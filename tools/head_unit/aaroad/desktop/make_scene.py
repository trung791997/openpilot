#!/usr/bin/env python3
"""A SYNTHETIC road network for the desktop dash-map renders. No real place: the origin is a tile in the open Atlantic and
every street is made up. Writes scene/scene.json: the road tile and traffic tile datagrams exactly as hu_feed sends them
(road_tiles.encode_road_tile, traffic_tiles.encode_traffic_tile, chunked and base64'd as hu_feed.chunk_tile does), and the
routes the renders drive, as lat/lon points.

Run from the repo root with numpy available, e.g.
  docker run --rm -v "$PWD":/w -w /w oprad-test:py312 python tools/head_unit/aaroad/desktop/make_scene.py
"""
import base64
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.getcwd())
from openpilot.starpilot.navigation import road_tiles as rt  # noqa: E402
from openpilot.starpilot.navigation import traffic_tiles as tt  # noqa: E402

TILE_CHUNK = 1000   # hu_feed.TILE_CHUNK
LAT0, LON0 = 30.0, -40.0   # open ocean; only picks the tile and the metres per tile
TX, TY = (int(v) for v in rt.world_xy(LAT0, LON0))
CX, CY = TX + 0.5, TY + 0.5  # the tile's centre is the local origin (east x, north y, metres)
MPT = rt.meters_per_tile(rt.lat_lon(CX, CY)[0])


def to_tile(pt):  # local metres -> tile units (0..4096), y down
  return [4096 * (0.5 + pt[0] / MPT), 4096 * (0.5 - pt[1] / MPT)]


def to_latlon(pt):
  return rt.lat_lon(CX + pt[0] / MPT, CY - pt[1] / MPT)


def dense(pts, step=12.0):
  out = [pts[0]]
  for a, b in zip(pts, pts[1:]):
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) // step))
    for i in range(1, n + 1):
      out.append([a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n])
  return out


def arc(cx, cy, r, a0, a1, n=24):  # degrees, 0 = east, counter-clockwise
  return [[cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))] for i in range(n + 1)]


def smooth(pts, n=10):  # Catmull-Rom through pts, for the bendy roads
  out = []
  p = [pts[0]] + pts + [pts[-1]]
  for i in range(1, len(p) - 2):
    for k in range(n):
      t = k / n
      out.append([0.5 * ((2 * p[i][d]) + (-p[i - 1][d] + p[i + 1][d]) * t + (2 * p[i - 1][d] - 5 * p[i][d] + 4 * p[i + 1][d] - p[i + 2][d]) * t * t
                         + (-p[i - 1][d] + 3 * p[i][d] - 3 * p[i + 1][d] + p[i + 2][d]) * t ** 3) for d in range(2)])
  out.append(pts[-1])
  return out


ROADS = {c: [] for c in range(5)}  # class -> [(points, name, flags, layer, lanes)]


def road(cls, pts, name="", flags=0, layer=0, lanes=0, step=12.0):
  ROADS[cls].append((dense(pts, step), name, flags, layer, lanes))


RING = (0.0, 260.0, 26.0)  # the roundabout: centre and radius, on Maple Avenue

# motorway: a gentle curve west to east, an overpass of Ridge Road over it, a ramp up to Maple Avenue
hw = smooth([[-1000, -430], [-600, -400], [-200, -385], [200, -385], [600, -405], [1000, -440]])
road(0, hw, "Harbor Freeway", flags=4, lanes=6, step=20)  # one-way flag only draws it as a single carriageway
road(0, [[-1000, -452], [-600, -422], [-200, -407], [200, -407], [600, -427], [1000, -462]], "Harbor Freeway", flags=4, lanes=6, step=20)
road(3, smooth([[-330, -400], [-250, -330], [-150, -230], [-40, -160]]), "Harbor Freeway", flags=4, lanes=1)
road(3, smooth([[40, -170], [140, -250], [260, -340], [340, -395]]), "Harbor Freeway", flags=4, lanes=1)
# arterials
road(1, [[0, -1000], [0, -160], [0, RING[1] - RING[2] - 4]], "Maple Avenue", lanes=4)
road(1, [[0, RING[1] + RING[2] + 4], [0, 1000]], "Maple Avenue", lanes=4)
road(1, [[-1000, -160], [-300, -160], [300, -160], [1000, -160]], "Commerce Boulevard", lanes=4)
road(1, [[-700, -380], [-700, -10], [-700, 600], [-700, 1000]], "Ridge Road", lanes=2)
road(1, [[640, -1000], [640, -480], [640, -420]], "Ridge Road", lanes=2)
road(1, [[640, -420], [640, -415], [640, -395]], "Ridge Road", flags=1, layer=1, lanes=2)  # the overpass above the freeway
road(1, [[640, -395], [640, -380], [640, 1000]], "Ridge Road", lanes=2)
road(1, smooth([[-1000, 700], [-600, 640], [-200, 660], [200, 640], [600, 700], [1000, 640]]), "Lakeshore Drive", lanes=3)
# the roundabout: the ring and its four arms (Cedar Street east and west)
road(2, arc(RING[0], RING[1], RING[2], 0, 360, 36), "", flags=4, lanes=2, step=4)
road(2, [[RING[0] + RING[2], RING[1]], [400, RING[1]], [900, RING[1]]], "Cedar Street", lanes=2)
road(2, [[RING[0] - RING[2], RING[1]], [-400, RING[1]], [-900, RING[1]]], "Cedar Street", lanes=2)
# streets
for x, nm in ((-420, "Oak Street"), (-210, "Pine Street"), (240, "Elm Street"), (440, "Birch Street")):
  road(2, [[x, -150], [x, 250], [x, 500]], nm, lanes=2)
for y, nm in ((-40, "First Street"), (100, "Second Street"), (430, "Third Street"), (560, "Fourth Street")):
  road(2, [[-900, y], [-300, y], [300, y], [900, y]], nm, lanes=2)
# minor: residential lanes and cul-de-sacs
for i, (x0, y0) in enumerate(((-340, 160), (110, 360), (330, 130), (-120, 480), (520, -80), (-560, 330))):
  road(4, [[x0, y0], [x0 + 70, y0 + 30], [x0 + 140, y0 + 20]], ["Birch Lane", "Willow Court", "Aspen Way", "Cedar Lane", "Holly Court", "Linden Way"][i], lanes=1, step=8)
road(4, arc(-560, 120, 30, 0, 330, 20), "Maple Court", lanes=1, step=6)
road(4, [[-530, 120], [-400, 120]], "Maple Court", lanes=1, step=8)

SIGNALS = [(0, -160), (-700, -160), (700, -160), (0, -40), (0, 100), (-420, -40), (240, 100), (0, 430), (640, 560), (-210, 560), (440, -40)]
RAILS = [(np.array([to_tile(p) for p in dense([[-1000, 820], [-300, 800], [300, 810], [1000, 830]])]), rt.MAJOR_RAIL)] if hasattr(rt, "MAJOR_RAIL") else []
WALKS = []
for x in (-3, 3):
  WALKS.append((np.array([to_tile([x * 4.5 + 0.0, y]) for y in range(-150, 230, 12)]), rt.SIDEWALK))


def build_tile():
  roads, names, flags, layers, lanes = {}, {}, {}, {}, {}
  for cls, items in ROADS.items():
    if not items:
      continue
    roads[cls] = [np.array([to_tile(p) for p in it[0]]) for it in items]
    names[cls] = [it[1] for it in items]
    flags[cls] = [it[2] for it in items]
    layers[cls] = [it[3] for it in items]
    lanes[cls] = [it[4] for it in items]
  pts = [(*to_tile(p), rt.SIGNAL) for p in SIGNALS]
  return rt.encode_road_tile(roads, names, flags=flags, points=pts, layers=layers, lanes=lanes, walks=WALKS, rails=RAILS)


def traffic():  # (level, local-metre polyline); levels 2 moderate, 3 heavy, 4 severe. Follows the roads above.
  out = []
  hw1 = [p for p in dense(hw, 12) if -150 <= p[0] <= 500]
  out.append((3, hw1))                                                 # the freeway, heavy
  out.append((4, [p for p in dense(hw, 12) if 500 < p[0] <= 700]))     # severe at the front of the jam
  out.append((2, [[0, -300], [0, -200], [0, -160], [0, -100]]))        # Maple Avenue south of the roundabout, moderate
  out.append((3, [[0, 300], [0, 400], [0, 500]]))                      # and north of it, heavy
  out.append((2, [[-300, -160], [-200, -160], [-100, -160]]))
  out.append((4, [[-210, -150], [-210, -60], [-210, 0]]))              # Pine Street, severe
  out.append((3, [[300, 100], [400, 100], [440, 100], [500, 100]]))
  out.append((2, [[900, RING[1]], [800, RING[1]], [650, RING[1]]]))
  return [(lvl, np.array([to_tile(p) for p in dense(pts, 15)])) for lvl, pts in out if len(pts) > 1]


def chunk(kind, key, raw):
  parts = [base64.b64encode(raw[i:i + TILE_CHUNK]).decode() for i in range(0, len(raw), TILE_CHUNK)]
  return [json.dumps({'v': 1, 't': kind, 'k': key, 'i': i, 'n': len(parts), 'b': part}, separators=(',', ':')) for i, part in enumerate(parts)]


def ring_route(entry_deg, exit_deg):  # counter-clockwise from the south arm entry to an exit angle, local metres
  cx, cy, r = RING
  a0 = -90.0
  return arc(cx, cy, r, a0, a0 + exit_deg, 24)


def scenarios():
  cx, cy, r = RING
  # roundabout: south on Maple up to the ring, round to the second exit (straight on, north), on up Maple
  south = [[0, -320], [0, cy - r]]
  ring = ring_route(0, 180)
  north = [[0, cy + r + 40 * i] for i in range(1, 12)]
  rpts = dense(south + ring[1:] + north, 12)
  # the maneuver is at the ring's entry; car starts 180 m before it
  man_s = 0.0
  for a, b in zip(rpts, rpts[1:]):
    if a[1] >= cy - r - 0.5 and abs(a[0]) < 1:
      break
    man_s += math.hypot(b[0] - a[0], b[1] - a[1])
  levels_for = lambda pts: [2 if -260 < p[1] < -150 else 2 if 110 < p[1] < 200 else 3 if 300 < p[1] < 420 else 4 if 420 <= p[1] < 480 else 0 for p in pts]
  rb = {"route": [to_latlon(p) for p in rpts], "levels": levels_for(rpts), "maneuver_s": man_s, "start_s": 0.0,
        "maneuver": {"type": "roundabout", "modifier": "straight", "exit": 2, "degrees": 180, "left": False,
                     "text": "At the roundabout, take the 2nd exit onto Maple Avenue", "road": "Maple Avenue", "ref": "", "toward": "Cedar Hills"},
        "road": "Maple Avenue", "remaining": 6400.0, "duration": 540.0, "limit": 17.9}
  # u-turn: north on Maple past the ring, turn round at a median gap 250 m on, back south on the other carriageway
  go = [[0, 520 + 15 * i] for i in range(0, 12)]
  turn = arc(0 - 9, go[-1][1], 9, 0, 180, 12)
  back = [[-18, go[-1][1] - 12 * i] for i in range(0, 15)]
  upts = dense([[-3, 330 + 12 * i] for i in range(0, 25)] + [[-3, 685]] + turn[1:] + back, 12)
  ut_s = 0.0
  for a, b in zip(upts, upts[1:]):
    if a[1] >= 685 - 0.5:
      break
    ut_s += math.hypot(b[0] - a[0], b[1] - a[1])
  ut = {"route": [to_latlon(p) for p in upts], "levels": [0] * len(upts), "maneuver_s": ut_s, "start_s": max(0.0, ut_s - 260.0),
        "maneuver": {"type": "turn", "modifier": "uturn", "exit": 0, "degrees": 0, "left": False,
                     "text": "Make a U-turn", "road": "Maple Avenue", "ref": "", "toward": ""},
        "road": "Maple Avenue", "remaining": 3100.0, "duration": 300.0, "limit": 17.9}
  # a free drive: along Commerce Boulevard east, then north on Elm Street, with traffic ahead on the road
  free = dense([[-300, -160], [240, -160], [240, 100], [240, 400]], 12)
  fr = {"track": [to_latlon(p) for p in free], "road": "Commerce Boulevard"}
  return {"roundabout": rb, "uturn": ut, "free": fr}


def main():
  key = f"{TX}/{TY}"
  tile = build_tile()
  assert rt.road_tile_version(tile) == rt.VERSION
  q = tt.encode_traffic_tile(traffic(), 0)
  out = {"origin": {"tile": key, "mpt": MPT}, "tiles": chunk('m', key, tile) + chunk('q', key, q), "scenarios": scenarios()}
  path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene", "scene.json")
  with open(path, "w") as f:
    json.dump(out, f, separators=(",", ":"))
  print(f"{path}: tile {key}, road tile {len(tile)} B, traffic {len(q)} B, {len(out['tiles'])} datagrams")


if __name__ == "__main__":
  main()
