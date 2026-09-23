"""Flag candidate segments from qlogs; fetch rlogs only for those.

usage: python tools/qlog_sweep.py ROUTE_DIR [ROUTE_DIR ...] --out OUT.json

ROUTE_DIR is <anything>/<route>/<seg>/qlog* (the tools/konik_fetch.py --qlogs layout). The whole route is
scanned as one stream and each flag is attributed to the segment where it starts. Times are route seconds,
(logMonoTime - min first logMonoTime over the route's segments) / 1e9.

qlogs only FIND candidates. Never run the radar parser on them: they do not carry the full CAN stream, so
parser results from a qlog are meaningless. Fetch the flagged segments' rlogs with
tools/konik_fetch.py --segments and replay those.

Flags:
  fcw          longitudinalPlan.fcw true (events < 2 s apart merged)
  hard_brake   engaged with carState.aEgo < -2.5 (gaps < 1 s merged)
  frozen_lead  a radar leadOne whose vRel and aLeadK stay bit-identical for >= 2 s while dRel moves >= 1 m
               (lead dropouts of <= 0.6 s bridged):
               the signature of a parser vRel coast (radard's KF is not stepped, STATUS 69)
  reversal     a radar leadOne with vRel >= +1 followed within 6 s by vRel <= -3 on the same track
"""
import argparse
import glob
import json
import os
import sys

sys.path.insert(0, '/src')
sys.path.insert(0, '/src/openpilot/tools')
from openpilot.tools.lib.logreader import _LogFileReader  # noqa: E402


def seg_files(route_dir):
  out = []
  for d in glob.glob(os.path.join(route_dir, '*')):
    files = glob.glob(os.path.join(d, 'qlog*'))
    if os.path.basename(d).isdigit() and files:
      out.append((int(os.path.basename(d)), files[0]))
  return sorted(out)


def scan_route(route_dir):
  segs = seg_files(route_dir)
  res = {str(s): {'engaged_s': 0.0, 'fcw': [], 'hard_brake': [], 'frozen_lead': [], 'reversal': [], 'error': None} for s, _ in segs}
  events = []
  for s, f in segs:
    try:
      for ev in _LogFileReader(f):
        w = ev.which()
        if w in ('selfdriveState', 'carState', 'longitudinalPlan', 'radarState'):
          events.append((ev.logMonoTime, s, w, getattr(ev, w)))
    except Exception as e:  # a truncated or corrupt segment must not end the sweep
      res[str(s)]['error'] = repr(e)[:200]
  if not events:
    return res
  events.sort(key=lambda e: e[0])
  t0 = min(e[0] for e in events)
  engaged, last_cs = False, None
  fcw_last = brake = frozen = frozen_raw = None
  opened = {}  # tid -> (t_open, seg)
  rev_last = {}
  for mono, s, w, m in events:
    t = round((mono - t0) / 1e9, 2)
    r = res[str(s)]
    if w == 'selfdriveState':
      engaged = bool(m.enabled)
    elif w == 'carState':
      if last_cs is not None and engaged:
        r['engaged_s'] += min(t - last_cs, 1.0)
      last_cs = t
      a = float(m.aEgo)
      if engaged and a < -2.5:
        if brake is not None and t - brake['t_end'] < 1.0:
          brake['a_min'] = min(brake['a_min'], round(a, 2))
          brake['t_end'] = t
        else:
          brake = {'t': t, 't_end': t, 'a_min': round(a, 2), 'v_ego': round(float(m.vEgo), 1)}
          r['hard_brake'].append(brake)
    elif w == 'longitudinalPlan':
      if m.fcw:
        if fcw_last is not None and t - fcw_last['t_last'] < 2.0:
          fcw_last['n'] += 1
          fcw_last['t_last'] = t
        else:
          fcw_last = {'t': t, 't_last': t, 'n': 1}
          r['fcw'].append(fcw_last)
    elif w == 'radarState':
      x = m.leadOne
      if not (x.status and x.radar):
        continue  # a lead dropout of <= 0.6 s may be bridged below; a longer one ends the run
      tid, v, ak, d = int(x.radarTrackId), float(x.vRel), float(x.aLeadK), float(x.dRel)
      # Compare against the raw previous values: the stored ones are rounded for the report.
      if frozen is not None and frozen['tid'] == tid and v == frozen_raw[0] and ak == frozen_raw[1] \
          and t - frozen['t_end'] <= 0.6:
        frozen['t_end'], frozen['d1'] = t, round(d, 1)
      else:
        frozen = {'t': t, 't_end': t, 'tid': tid, 'd0': round(d, 1), 'd1': round(d, 1), 'vrel': round(v, 2),
                  'aleadk': round(ak, 2)}
        frozen_raw = (v, ak)
        r['frozen_lead'].append(frozen)
      if v >= 1.0:
        opened[tid] = (t, s)
      elif v <= -3.0 and tid in opened and t - opened[tid][0] <= 6.0 and t - rev_last.get(tid, -1e9) > 6.0:
        rev_last[tid] = t
        res[str(opened[tid][1])]['reversal'].append({'t_open': opened[tid][0], 't_close': t, 'tid': tid, 'd': round(d, 1)})
  for r in res.values():
    for b in r['hard_brake']:
      b['dur'] = round(b.pop('t_end') - b['t'], 2)
    for f in r['fcw']:
      f.pop('t_last')
    kept = []
    for f in r['frozen_lead']:
      f['dur'] = round(f.pop('t_end') - f['t'], 2)
      if f['dur'] >= 2.0 and abs(f['d1'] - f['d0']) >= 1.0:
        kept.append(f)
    r['frozen_lead'] = kept
    r['engaged_s'] = round(r['engaged_s'], 1)
  return res


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('route_dirs', nargs='+')
  ap.add_argument('--out', required=True)
  a = ap.parse_args()
  out = {}
  for rd in a.route_dirs:
    route = os.path.basename(os.path.normpath(rd))
    out[route] = {'segments': scan_route(rd)}
    for s, r in out[route]['segments'].items():
      flags = {k: r[k] for k in ('fcw', 'hard_brake', 'frozen_lead', 'reversal') if r[k]}
      if flags or r['error']:
        print(route, s, r['engaged_s'], {k: len(v) for k, v in flags.items()}, json.dumps(flags), r['error'] or '')
  json.dump(out, open(a.out, 'w'))


if __name__ == '__main__':
  main()
