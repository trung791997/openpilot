#!/usr/bin/env python3
"""Run ON THE COMMA (offroad only): write <route>_comma.csv (gpsLocationExternal), <route>_hu.csv
(starpilotNavigation.headUnitGps, the car GPS) and <route>_car.csv (carState vEgo, 10 Hz) to /tmp/gpsx from rlogs.
Usage: cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python extract.py ROUTE [ROUTE...]
Copy /tmp/gpsx/*.csv off the device, delete /tmp/gpsx, then run analyze.py DIR and figure.py DIR ROUTE MM:SS-MM:SS."""

import sys
import glob
import os
import csv
import zstandard
from openpilot.cereal import log

base = '/data/media/0/realdata'
out = '/tmp/gpsx'
os.makedirs(out, exist_ok=True)


def f(x):
  return '' if x != x else f'{x:.9g}'


for r in sys.argv[1:]:
  if os.path.exists('/data/params/d/IsOnroad'):
    print('ONROAD stop')
    sys.exit(1)
  segs = sorted(glob.glob(f'{base}/{r}--*'), key=lambda p: int(p.rsplit('--', 1)[1]))
  fc = open(f'{out}/{r}_comma.csv', 'w')
  fh = open(f'{out}/{r}_hu.csv', 'w')
  fv = open(f'{out}/{r}_car.csv', 'w')
  wc, wh, wv = csv.writer(fc), csv.writer(fh), csv.writer(fv)
  wc.writerow(['mono', 'unix_ms', 'lat', 'lon', 'speed', 'bearing', 'hacc', 'bacc', 'sats', 'flags'])
  wh.writerow(['mono', 'src_mono', 'lat', 'lon', 'speed', 'bearing', 'hacc'])
  wv.writerow(['mono', 'vEgo', 'yawRate', 'standstill'])
  for s in segs:
    p = f'{s}/rlog.zst'
    if not os.path.exists(p):
      continue
    try:
      d = zstandard.ZstdDecompressor().stream_reader(open(p, 'rb')).read()
      n = 0
      for m in log.Event.read_multiple_bytes(d):
        w = m.which()
        if w == 'gpsLocationExternal':
          g = m.gpsLocationExternal
          wc.writerow(
            [
              m.logMonoTime,
              g.unixTimestampMillis,
              f'{g.latitude:.8f}',
              f'{g.longitude:.8f}',
              f(g.speed),
              f(g.bearingDeg),
              f(g.horizontalAccuracy),
              f(g.bearingAccuracyDeg),
              g.satelliteCount,
              g.flags,
            ]
          )
        elif w == 'starpilotNavigation':
          g = m.starpilotNavigation.headUnitGps
          if g.sourceMonoTime:
            wh.writerow([m.logMonoTime, g.sourceMonoTime, f'{g.latitude:.8f}', f'{g.longitude:.8f}', f(g.speed), f(g.bearingDeg), f(g.horizontalAccuracy)])
        elif w == 'carState':
          n += 1
          if n % 10 == 0:
            c = m.carState
            wv.writerow([m.logMonoTime, f(c.vEgo), f(c.yawRate), int(c.standstill)])
    except Exception as e:
      print(s, 'err', e)
  for x in (fc, fh, fv):
    x.close()
  print(r, 'done', flush=True)
