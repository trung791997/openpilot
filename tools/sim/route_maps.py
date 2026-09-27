#!/usr/bin/env python3
"""Sim maps of the roads the owner drives most (SIM_MAP=route, SIM_MAP_FILE).

  list   OUT_DIR                 route listing from Konik -> OUT_DIR/routes.json; prints the most frequent trips
  track  QLOG_ROUTE_DIR OUT.npz  speed, yaw rate and engagement from a route's qlogs
  blocks TRACK.npz OUT.json      straights and constant-radius curves for metadrive_bridge.route_map_blocks

Coordinates are home/work locations: keep routes.json and the tracks outside the repo (~/.openpilot-sim/civic/routes).
Only the block files (lengths, radii, angles; no position) belong in tools/sim/maps. Places print as P1, P2, ...
"""
import json
import math
import os
import sys
from collections import Counter
from datetime import datetime, timezone

EARTH_R = 6371000.0
PLACE_M = 400.0  # a trip end within this of a known place is that place (parking spots differ)


def _xy(lat, lng, lat0, lng0):
  return (math.radians(lng - lng0) * EARTH_R * math.cos(math.radians(lat0)), math.radians(lat - lat0) * EARTH_R)


def cmd_list(out_dir, days=365):
  sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
  import plain_http
  from konik_preflight import KONIK_API_HOST, _token
  host = os.environ.get("API_HOST", KONIK_API_HOST).rstrip("/")
  hdr = {"Authorization": f"JWT {_token(host)}", "User-Agent": "sim-route-maps"}

  def api(path):
    code, text, _ = plain_http.request("GET", f"{host}/{path}", headers=hdr, timeout=60)
    if code >= 400:
      raise RuntimeError(f"{code} on /{path}")
    return json.loads(text)

  dongle = os.environ.get("DONGLE_ID") or api("v1/me/devices/")[0]["dongle_id"]
  end = int(datetime.now(timezone.utc).timestamp() * 1000)
  routes = api(f"v1/devices/{dongle}/routes_segments?start={end - days * 86400000}&end={end}") or []
  os.makedirs(out_dir, exist_ok=True)
  with open(os.path.join(out_dir, "routes.json"), "w") as f:
    json.dump(routes, f)

  places, trips = [], Counter()
  examples = {}

  def place(lat, lng):
    for i, (plat, plng) in enumerate(places):
      if math.hypot(*_xy(lat, lng, plat, plng)) < PLACE_M:
        return i
    places.append((lat, lng))
    return len(places) - 1

  for r in sorted(routes, key=lambda r: r.get("start_time_utc_millis", 0)):
    if not r.get("start_lat") or not r.get("end_lat"):
      continue
    a, b = place(r["start_lat"], r["start_lng"]), place(r["end_lat"], r["end_lng"])
    key = (a, b)
    trips[key] += 1
    km = r.get("length") or 0  # miles in the comma API
    examples.setdefault(key, []).append((r.get("fullname"), round(km * 1.609, 1), r.get("maxqlog", -1) + 1))
  with open(os.path.join(out_dir, "places.json"), "w") as f:
    json.dump({"places": places, "trips": [[a, b, n, examples[(a, b)]] for (a, b), n in trips.most_common()]}, f)
  print(f"{len(routes)} routes, {len(places)} places, {len(trips)} distinct trips")
  for (a, b), n in trips.most_common(15):
    ex = examples[(a, b)]
    print(f"P{a + 1} -> P{b + 1}: {n} drives, {sorted(e[1] for e in ex)[len(ex) // 2]} km median; latest {ex[-1][0]}")


def cmd_track(route_dir, out):
  """Speed (carState), yaw rate (livePose, device z points down so left = -z) and engagement per message time. The
  path is dead-reckoned from these: qlogs carry GPS at about 1 Hz, too coarse and noisy for curve radii."""
  import numpy as np
  from openpilot.tools.lib.logreader import LogReader
  segs = sorted((d for d in os.listdir(route_dir) if d.isdigit()), key=int)
  cols = {k: [] for k in ("t_v", "v", "t_yaw", "yaw", "t_eng", "eng")}
  for s in segs:
    p = next((os.path.join(route_dir, s, n) for n in ("qlog.zst", "qlog.bz2", "qlog", "rlog.zst", "rlog.bz2", "rlog")
              if os.path.exists(os.path.join(route_dir, s, n))), None)
    if p is None:
      continue
    for m in LogReader(p):
      w, t = m.which(), m.logMonoTime * 1e-9
      if w == "carState":
        cols["t_v"].append(t); cols["v"].append(m.carState.vEgo)
      elif w == "livePose":
        cols["t_yaw"].append(t); cols["yaw"].append(-m.livePose.angularVelocityDevice.z)
      elif w == "selfdriveState":
        cols["t_eng"].append(t); cols["eng"].append(m.selfdriveState.active)
  np.savez(out, **{k: np.array(v) for k, v in cols.items()})
  print(f"{len(segs)} segments, {len(cols['v'])} speeds, {len(cols['yaw'])} yaw rates -> {out}")


def track_to_blocks(x, y, straight_curv=1 / 1500, min_angle_deg=5.0, step_m=2.0, gap_m=20.0, edge_m=10.0):
  """Straights and constant-radius curves from a path (metres). Curves are runs of one curvature sign above
  straight_curv (gaps under gap_m merged). A curve's angle is the heading change across it (edge_m either side), its
  radius from its tightest half (1 / median |curvature| where it is above half its peak), and its arc length R * angle;
  the rest of the run's length goes to the straights either side, so the total distance is kept.
  Returns [["S", length], ["C", radius, angle_deg, dir], ...], dir 0 = left (as civic_corners.json)."""
  import numpy as np
  d = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
  s = np.arange(0, d[-1], step_m)
  k = np.ones(11) / 11  # ~20 m: GPS noise and lane changes
  xs = np.convolve(np.interp(s, d, x), k, "valid")
  ys = np.convolve(np.interp(s, d, y), k, "valid")
  h, w = 5, 3  # heading over a 20 m chord, curvature over 12 m: a 2 m difference turns GPS noise into curvature noise
  hd = np.unwrap(np.arctan2(ys[2 * h:] - ys[:-2 * h], xs[2 * h:] - xs[:-2 * h]))
  curv = np.r_[[0.0] * w, (hd[2 * w:] - hd[:-2 * w]) / (2 * w * step_m), [0.0] * w]  # + = left (counter-clockwise)
  n = len(hd)
  sign = np.where(np.abs(curv) < straight_curv, 0, np.sign(curv)).astype(int)
  runs, i = [], 0
  while i < n:
    j = i
    while j < n and sign[j] == sign[i]:
      j += 1
    if sign[i] != 0:
      if runs and runs[-1][2] == sign[i] and (i - runs[-1][1]) * step_m < gap_m:
        runs[-1][1] = j
      else:
        runs.append([i, j, sign[i]])
    i = j
  e = int(edge_m / step_m)
  curves = []
  for i, j, sg in runs:
    turn = hd[min(j + e, n - 1)] - hd[max(i - e, 0)]
    if abs(math.degrees(turn)) < min_angle_deg:
      continue
    c = np.abs(curv[i:j])
    r = 1.0 / float(np.median(c[c >= 0.5 * c.max()]))
    curves.append((i * step_m, j * step_m, r, abs(turn), 0 if turn > 0 else 1))
  blocks, pos = [], 0.0
  for a0, a1, r, turn, dr in curves:
    arc = r * turn
    mid = (a0 + a1) / 2
    start = max(mid - arc / 2, pos)
    if start - pos >= 1:
      blocks.append(["S", round(start - pos)])
    blocks.append(["C", round(r, 1), round(math.degrees(turn)), dr])
    pos = start + arc
  if n * step_m - pos >= 1:
    blocks.append(["S", round(n * step_m - pos)])
  return blocks


def cmd_blocks(track, out, max_km=0.9):
  """Blocks for a trip, split into pieces of at most max_km (MetaDrive paints a bounded area and cannot overlap
  blocks: its lane lines are painted inside a 1024 m square). Each piece's typical engaged speed goes to OUT_DIR/drives.json for SIM_CRUISE_KPH."""
  import numpy as np
  z = np.load(track)
  t = np.arange(z["t_v"][0], z["t_v"][-1], 0.1)
  v = np.interp(t, z["t_v"], z["v"])
  yaw = np.interp(t, z["t_yaw"], z["yaw"])
  eng = np.interp(t, z["t_eng"], z["eng"].astype(float)) > 0.5
  mv = v > 1.0  # parked and creeping: heading drifts with the gyro bias, distance is nil
  hd = np.cumsum(np.where(mv, yaw, 0.0) * 0.1)
  x, y, sdist = np.cumsum(v * np.cos(hd) * 0.1)[mv], np.cumsum(v * np.sin(hd) * 0.1)[mv], np.cumsum(v * 0.1)[mv]
  vm, em = v[mv], eng[mv]
  blocks = track_to_blocks(x, y)
  pieces, cur, dist, bounds = [], [], 0.0, [0.0]
  split = []
  for b in blocks:  # a straight longer than a piece is cut so every piece stays under max_km
    while b[0] == "S" and b[1] > max_km * 500:
      split.append(["S", round(max_km * 500)]); b = ["S", b[1] - round(max_km * 500)]
    split.append(b)
  for b in split:
    ln = b[1] if b[0] == "S" else b[1] * math.radians(b[2])
    if cur and dist + ln > max_km * 1000:
      pieces.append(cur); cur = []; bounds.append(bounds[-1] + dist); dist = 0.0
    cur.append(b); dist += ln
  pieces.append(cur); bounds.append(bounds[-1] + dist)
  base = out[:-5] if out.endswith(".json") else out
  idx_path = os.path.join(os.path.dirname(base) or ".", "drives.json")
  idx = json.load(open(idx_path)) if os.path.exists(idx_path) else {}
  for n, p in enumerate(pieces):
    fn = f"{base}_{n + 1}.json" if len(pieces) > 1 else f"{base}.json"
    with open(fn, "w") as f:
      f.write("[" + ",\n ".join(json.dumps(b) for b in [["S", 60]] + p) + "]\n")
    sel = (sdist >= bounds[n]) & (sdist < bounds[n + 1])
    kph = float(np.median(vm[sel & em] if (sel & em).sum() > 50 else vm[sel])) * 3.6
    km = (bounds[n + 1] - bounds[n]) / 1000
    # episode length: the piece at its speed plus ~25 s to engage and reach the set speed, before the road runs out
    idx[os.path.basename(fn)] = {"cruise_kph": round(kph), "engaged_share": round(float(em[sel].mean()), 2),
                                 "km": round(km, 2), "episode_s": int(km / max(kph, 20) * 3600 + 25)}
    curves = [b for b in p if b[0] == "C"]
    print(f"{fn}: {len(p)} blocks, {len(curves)} curves, min R {min((b[1] for b in curves), default=None)}, "
          f"{(bounds[n + 1] - bounds[n]) / 1000:.1f} km, {round(kph)} km/h, engaged {em[sel].mean():.0%}")
  with open(idx_path, "w") as f:
    json.dump(idx, f, indent=1, sort_keys=True)


if __name__ == "__main__":
  cmd = sys.argv[1] if len(sys.argv) > 1 else ""
  if cmd == "list":
    cmd_list(sys.argv[2])
  elif cmd == "track":
    cmd_track(sys.argv[2], sys.argv[3])
  elif cmd == "blocks":
    cmd_blocks(sys.argv[2], sys.argv[3])
  else:
    print(__doc__)
