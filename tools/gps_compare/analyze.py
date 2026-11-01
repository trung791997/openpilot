#!/usr/bin/env python3
"""Compare the comma's GNSS (gpsLocationExternal) with the car's GPS (starpilotNavigation.headUnitGps).

Inputs: <route>_comma.csv, <route>_hu.csv, <route>_car.csv in DIR (from extract.py). Roads come from OpenStreetMap
(Overpass), cached as osm_<route>.json. Prints per-route and pooled stats; writes stats.json.
"""

import csv
import json
import math
import os
import sys
import time
import urllib.parse
import urllib.request
import numpy as np

DIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp/gpscmp"
ROUTES = sys.argv[2:] or sorted({f.rsplit("_", 1)[0] for f in os.listdir(DIR) if f.endswith("_hu.csv")})
MIN_SPEED = 5.0  # m/s; drops stops and parking lots
MATCH_RADIUS = 60.0  # m; a fix farther than this from every road counts as that far (capped)
HEADING_GATE = 35.0  # deg; prefer road segments aligned with the fix's own bearing
HIGHWAYS = (
  "motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link"
)
R_EARTH = 6378137.0


def load(path):
  rows = list(csv.reader(open(path)))
  a = np.array([[float(x) if x != "" else np.nan for x in r] for r in rows[1:]]).reshape(-1, len(rows[0]))
  return {k: a[:, i] for i, k in enumerate(rows[0])}


class Proj:
  def __init__(self, lat0, lon0):
    self.lat0, self.lon0, self.k = lat0, lon0, math.cos(math.radians(lat0))

  def __call__(self, lat, lon):
    return (np.radians(np.asarray(lon) - self.lon0) * R_EARTH * self.k, np.radians(np.asarray(lat) - self.lat0) * R_EARTH)


def overpass(route, lat, lon):
  cache = f"{DIR}/osm_{route}.json"
  if os.path.exists(cache):
    return json.load(open(cache))
  pts, last = [], None
  for la, lo in zip(lat, lon, strict=True):
    if not (math.isfinite(la) and math.isfinite(lo)) or la == 0:
      continue
    # thin the track to ~150 m spacing for an around: polyline
    if last is None or math.hypot((la - last[0]) * 111e3, (lo - last[1]) * 93e3) > 150:
      pts.append((la, lo))
      last = (la, lo)
  ways = {}
  for i in range(0, len(pts), 400):
    chunk = pts[max(0, i - 1) : i + 400]
    line = ",".join(f"{a:.5f},{b:.5f}" for a, b in chunk)
    q = f'[out:json][timeout:120];(way[highway~"^({HIGHWAYS})$"](around:120,{line});way[bridge](around:120,{line}););out tags geom;'
    req = urllib.request.Request("https://overpass-api.de/api/interpreter", urllib.parse.urlencode({"data": q}).encode(), headers={"User-Agent": "gpscmp/1.0"})
    for attempt in range(4):
      try:
        reply = json.load(urllib.request.urlopen(req, timeout=180))
        elements = reply["elements"]
        if not elements:
          raise RuntimeError(reply.get("remark", "empty reply"))
        break
      except Exception as e:  # Overpass rate limits with 429/504; wait and retry
        print("overpass retry", attempt, e, file=sys.stderr)
        time.sleep(30)
    else:
      raise RuntimeError("Overpass failed")
    for w in elements:
      ways[w["id"]] = {"tags": w.get("tags", {}), "geom": [(g["lat"], g["lon"]) for g in w.get("geometry", [])]}
  if not ways:
    raise RuntimeError("Overpass returned no roads")
  json.dump(ways, open(cache, "w"))
  return ways


class Roads:
  def __init__(self, ways, proj):
    segs, info = [], []
    for wid, w in ways.items():
      t = w["tags"]
      hw = t.get("highway", "")
      if not hw or hw in ("footway", "path", "cycleway", "pedestrian", "steps", "track") or t.get("service") in ("parking_aisle", "driveway", "drive-through"):
        continue
      g = np.array(w["geom"])
      x, y = proj(g[:, 0], g[:, 1])
      for j in range(len(x) - 1):
        segs.append((x[j], y[j], x[j + 1], y[j + 1]))
        info.append(wid)
    self.s = np.array(segs)
    self.wid = np.array(info, dtype=object)
    self.az = np.degrees(np.arctan2(self.s[:, 2] - self.s[:, 0], self.s[:, 3] - self.s[:, 1])) % 180
    self.cell = 100.0
    self.grid = {}
    for i, (x0, y0, x1, y1) in enumerate(self.s):
      for cx in range(int(min(x0, x1) // self.cell) - 1, int(max(x0, x1) // self.cell) + 2):
        for cy in range(int(min(y0, y1) // self.cell) - 1, int(max(y0, y1) // self.cell) + 2):
          self.grid.setdefault((cx, cy), []).append(i)

  def match(self, x, y, bearing):
    """Cross-track distance (capped), segment azimuth and way id for each point."""
    d = np.full(len(x), MATCH_RADIUS)
    az = np.full(len(x), np.nan)
    wid = np.full(len(x), None, dtype=object)
    for k in range(len(x)):
      if not np.isfinite(x[k]):
        d[k] = np.nan
        continue
      c = self.grid.get((int(x[k] // self.cell), int(y[k] // self.cell)))
      if not c:
        continue
      s = self.s[c]
      dx, dy = s[:, 2] - s[:, 0], s[:, 3] - s[:, 1]
      t = np.clip(((x[k] - s[:, 0]) * dx + (y[k] - s[:, 1]) * dy) / np.maximum(dx * dx + dy * dy, 1e-9), 0, 1)
      dist = np.hypot(s[:, 0] + t * dx - x[k], s[:, 1] + t * dy - y[k])
      if np.isfinite(bearing[k]):
        diff = np.abs((bearing[k] - self.az[c] + 90) % 180 - 90)
        aligned = diff <= HEADING_GATE
        if aligned.any() and dist[aligned].min() <= MATCH_RADIUS:
          dist = np.where(aligned, dist, np.inf)
      j = int(np.argmin(dist))
      if dist[j] < MATCH_RADIUS:
        d[k], az[k], wid[k] = dist[j], self.az[c][j], self.wid[c][j]
    return d, az, wid


def pct(a, q=(50, 95, 100)):
  a = np.asarray(a, float)
  a = a[np.isfinite(a)]
  return [round(float(np.percentile(a, p)), 2) for p in q] if len(a) else [None] * len(q)


def mmss(sec):
  return f"{int(sec // 60):02d}:{int(sec % 60):02d}"


def analyze(route):
  c, h, v = (load(f"{DIR}/{route}_{k}.csv") for k in ("comma", "hu", "car"))
  t0 = v["mono"][0]
  _, iu = np.unique(h["src_mono"], return_index=True)  # navigationd republishes a held fix; keep each fix once
  h = {k: a[iu] for k, a in h.items()}
  h["t"] = h["src_mono"]
  c["t"] = c["mono"]
  c["ok"] = (c["flags"] % 2 == 1) & (c["hacc"] <= 25) & (c["lat"] != 0)
  h["ok"] = np.isfinite(h["lat"]) & (h["hacc"] <= 25)
  lat0, lon0 = np.nanmedian(h["lat"]), np.nanmedian(h["lon"])
  proj = Proj(lat0, lon0)
  roads = Roads(overpass(route, h["lat"], h["lon"]), proj)
  out = {"route": route, "duration_s": round((v["mono"][-1] - t0) / 1e9)}
  for _name, s in (("comma", c), ("car", h)):
    s["x"], s["y"] = proj(np.where(s["lat"] != 0, s["lat"], np.nan), np.where(s["lat"] != 0, s["lon"], np.nan))
    s["v"] = np.interp(s["t"], v["mono"], v["vEgo"])
    s["mv"] = s["v"] > MIN_SPEED
    s["xt"], s["az"], s["wid"] = roads.match(s["x"], s["y"], np.where(s["v"] > 3, s["bearing"], np.nan))
  # time with usable fixes while moving, on a 1 s grid
  grid = np.arange(v["mono"][0], v["mono"][-1], 1e9)
  vg = np.interp(grid, v["mono"], v["vEgo"])
  mvg = vg > MIN_SPEED
  for name, s in (("comma", c), ("car", h)):
    ts = s["t"][s["ok"]]
    idx = np.searchsorted(ts, grid)
    age = np.where(idx > 0, (grid - ts[np.clip(idx - 1, 0, max(len(ts) - 1, 0))]) / 1e9, np.inf) if len(ts) else np.full(len(grid), np.inf)
    fresh = age <= 1.5
    dt = np.diff(s["t"]) / 1e9
    gaps = [
      (mmss((s["t"][i] - t0) / 1e9), round(float(dt[i]), 1))
      for i in np.nonzero(dt > (3 if name == "car" else 1))[0]
      if np.interp(s["t"][i], v["mono"], v["vEgo"]) > 1
    ]
    sel = s["mv"] & s["ok"]
    r = {
      "moving_fixes": int(s["mv"].sum()),
      "usable_frac_moving_time": round(float(fresh[mvg].mean()), 3),
      "rate_hz": round(1 / float(np.median(dt)), 2),
      "gaps_while_driving": len(gaps),
      "longest_gaps": sorted(gaps, key=lambda g: -g[1])[:3],
      "reported_hacc_p50_p95": pct(s["hacc"][s["mv"] & s["ok"]], (50, 95)),
      "xtrack_usable_p50_p95_max": pct(s["xt"][sel]),
      "xtrack_all_moving_p50_p95_max": pct(s["xt"][s["mv"]]),
      "n_usable_moving": int(sel.sum()),
    }
    # heading vs matched road azimuth (folded, speed > 8 m/s)
    hs = sel & (s["v"] > 8) & np.isfinite(s["az"]) & np.isfinite(s["bearing"])
    r["heading_err_deg_p50_p95"] = pct(np.abs((s["bearing"][hs] - s["az"][hs] + 90) % 180 - 90), (50, 95))
    # speed vs wheel speed
    r["speed_minus_wheel_p50abs_p95abs"] = pct(np.abs(s["speed"][sel] - s["v"][sel]), (50, 95))
    # jumpiness: 1 s position step vs wheel-speed distance
    step = 10 if name == "comma" else 1
    a, b = np.arange(len(s["t"]) - step), np.arange(step, len(s["t"]))
    dtt = (s["t"][b] - s["t"][a]) / 1e9
    good = s["ok"][a] & s["ok"][b] & s["mv"][a] & (dtt > 0.8) & (dtt < 1.25)
    stepd = np.hypot(s["x"][b] - s["x"][a], s["y"][b] - s["y"][a])
    vm = np.array([np.mean(np.interp(np.linspace(s["t"][i], s["t"][j], 5), v["mono"], v["vEgo"])) for i, j in zip(a[good], b[good], strict=True)])
    res = stepd[good] - vm * dtt[good]
    r["step_residual_m_p50abs_p95abs_max"] = pct(np.abs(res - np.median(res)) if len(res) else [], (50, 95, 100))
    r["step_bias_m"] = round(float(np.median(res)), 2) if len(res) else None
    # lag vs wheel speed: shift that best lines the source speed up with vEgo
    if sel.sum() > 200:
      taus = np.arange(-2.0, 3.01, 0.05)
      err = [np.nanmean(np.abs(s["speed"][sel] - np.interp(s["t"][sel] - tau * 1e9, v["mono"], v["vEgo"]))) for tau in taus]
      r["speed_lag_vs_wheels_s"] = round(float(taus[int(np.argmin(err))]), 2)
    worst = np.argsort(-np.where(sel, s["xt"], -1))[:5]
    r["worst_xtrack"] = [(mmss((s["t"][i] - t0) / 1e9), round(float(s["xt"][i]), 1)) for i in worst if sel[i]]
    out[name] = r
  # head-to-head where both usable: distance between sources, and relative time lag
  cm = c["ok"] & np.isfinite(c["x"])
  if cm.sum() > 300 and h["ok"].sum() > 100:
    tc = c["t"][cm]
    near = np.abs(tc[np.clip(np.searchsorted(tc, h["t"]), 0, len(tc) - 1)] - h["t"]) < 0.25e9
    both = h["ok"] & near & h["mv"]
    if both.sum() > 60:
      taus = np.arange(-2.0, 2.01, 0.05)
      med = []
      for tau in taus:
        xi = np.interp(h["t"][both] + tau * 1e9, tc, c["x"][cm])
        yi = np.interp(h["t"][both] + tau * 1e9, tc, c["y"][cm])
        med.append(np.median(np.hypot(xi - h["x"][both], yi - h["y"][both])))
      k = int(np.argmin(med))
      out["head_to_head"] = {
        "n": int(both.sum()),
        "dist_m_at_0_lag_p50": round(float(med[len(taus) // 2]), 2),
        "best_tau_s": round(float(taus[k]), 2),
        "dist_m_at_best_p50": round(float(med[k]), 2),
      }
      xi = np.interp(h["t"][both], tc, c["x"][cm])
      yi = np.interp(h["t"][both], tc, c["y"][cm])
      dd = np.hypot(xi - h["x"][both], yi - h["y"][both])
      out["head_to_head"]["dist_m_p95_max"] = pct(dd, (95, 100))
      # paired cross-track at the same instants
      cx = np.interp(h["t"][both], tc, c["x"][cm])
      cy = np.interp(h["t"][both], tc, c["y"][cm])
      pxt, _, _ = roads.match(cx, cy, h["bearing"][both])
      out["head_to_head"]["paired_xtrack_comma_p50_p95"] = pct(pxt, (50, 95))
      out["head_to_head"]["paired_xtrack_car_p50_p95"] = pct(h["xt"][both], (50, 95))
  # passing under bridges: car-track segments that cross a bridge way not being driven
  bridges = [w for w in roads_ways(route) if w["tags"].get("bridge") not in (None, "no")]
  under = []
  for w in bridges:
    g = np.array(w["geom"])
    bx, by = proj(g[:, 0], g[:, 1])
    for j in range(len(bx) - 1):
      for i in np.nonzero(h["mv"][:-1])[0]:
        if crosses(h["x"][i], h["y"][i], h["x"][i + 1], h["y"][i + 1], bx[j], by[j], bx[j + 1], by[j + 1]):
          under.append(h["t"][i])
  under = sorted({round(u / 5e9) * 5e9 for u in under})
  if under:

    def near_u(t):
      return np.min(np.abs(np.asarray(t)[:, None] - np.array(under)[None, :]), axis=1) < 6e9

    ub = {}
    for name, s in (("comma", c), ("car", h)):
      m = s["mv"] & near_u(s["t"])
      ub[name] = {"usable_frac": round(float(s["ok"][m].mean()), 2) if m.any() else None, "xtrack_p50_p95": pct(s["xt"][m & s["ok"]], (50, 95))}
    out["under_bridges"] = {"crossings": len(under), "times": [mmss((u - t0) / 1e9) for u in under[:12]], **ub}
  return out, (c, h, v, roads, proj, t0)


_ways = {}


def roads_ways(route):
  if route not in _ways:
    _ways[route] = list(json.load(open(f"{DIR}/osm_{route}.json")).values())
  return _ways[route]


def crosses(ax, ay, bx, by, cx, cy, dx, dy):
  def o(px, py, qx, qy, rx, ry):
    return (qx - px) * (ry - py) - (qy - py) * (rx - px)

  if not all(np.isfinite([ax, ay, bx, by])):
    return False
  return o(ax, ay, bx, by, cx, cy) * o(ax, ay, bx, by, dx, dy) < 0 and o(cx, cy, dx, dy, ax, ay) * o(cx, cy, dx, dy, bx, by) < 0


if __name__ == "__main__":
  results = []
  for r in ROUTES:
    res, _ = analyze(r)
    results.append(res)
    print(json.dumps(res, default=str))
  json.dump(results, open(f"{DIR}/stats.json", "w"), indent=1, default=str)
