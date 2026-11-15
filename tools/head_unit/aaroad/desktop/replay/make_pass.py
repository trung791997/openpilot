#!/usr/bin/env python3
"""gps_<route>_<seg>.csv files (all gpsLocationExternal/gpsLocation fixes of a qlog segment, from the scratchpad sweep) ->
pass files for Replay: one per route, the contiguous stretch within RADIUS m of the crossing, as t_ms,lat,lon,bearing,speed.
Usage: make_pass.py <tracks dir> <out dir> [radius_m]. Plain python3, no numpy."""
import csv, glob, math, os, re, sys

LA, LO = 33.1168, -96.6703   # the Exchange Pkwy / US-75 crossing (centre of the deck, from the road tile geometry)
src, out = sys.argv[1], sys.argv[2]
R = float(sys.argv[3]) if len(sys.argv) > 3 else 600
os.makedirs(out, exist_ok=True)
by_route = {}
for f in sorted(glob.glob(os.path.join(src, 'gps_*.csv'))):
  m = re.match(r'gps_(.+)_(\d+)\.csv', os.path.basename(f))
  by_route.setdefault(m.group(1), []).append((int(m.group(2)), f))
for route, segs in sorted(by_route.items()):
  fixes = []
  for _, f in sorted(segs):
    for r in csv.DictReader(open(f)):
      if r['src'] == 'gpsLocationExternal' and r['hasFix'] == 'True':
        fixes.append((int(r['logMonoTime']) / 1e6, float(r['lat']), float(r['lon']), float(r['bearing']), float(r['speed'])))
  fixes.sort()
  def dist(x): return math.hypot((x[1] - LA) * 111000, (x[2] - LO) * 93000)
  # split into windows of consecutive fixes (gap < 3 s) inside R
  win, cur = [], []
  for x in fixes:
    if dist(x) < R and (not cur or x[0] - cur[-1][0] < 3000):
      cur.append(x)
    else:
      if cur: win.append(cur)
      cur = [x] if dist(x) < R else []
  if cur: win.append(cur)
  for k, w in enumerate(win):
    if len(w) < 10: continue
    name = os.path.join(out, 'pass_%s_%d.csv' % (route.split('--')[0], k))
    with open(name, 'w') as fh:
      fh.write('t_ms,lat,lon,bearing_deg,speed_mps\n')
      for x in w: fh.write('%.0f,%.7f,%.7f,%.1f,%.2f\n' % x)
    print(name, len(w), 'fixes, closest %.0f m' % min(dist(x) for x in w))
