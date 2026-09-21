# Sweep-by-sweep trace of an arbitrary route-time window, same shape as diag936.py.
# Usage: python diagwin.py <route> <A_sec> <B_sec> [track_id]
# Prints EVERY sweep in the window -- diag936.py printed out[::2], which made the
# 241 loss time read 9:35.86 instead of the true 9:35.80.
import sys, json, glob, math
sys.path.insert(0,'/src'); sys.path.insert(0,'/src/openpilot/tools')
from openpilot.tools.lib.logreader import _LogFileReader
from opendbc.car.can_definitions import CanData
import opendbc.car.honda.radar_interface as RI
from bosch_a_route_report import build_radar_interface

r = sys.argv[1]; A = float(sys.argv[2]); B = float(sys.argv[3])
TID = int(sys.argv[4]) if len(sys.argv) > 4 else None

d = json.load(open(f'/routes/an2/scan_{r}.json'))
seg_t0 = {int(k): int(v) for k, v in d['seg_t0'].items()}
T0 = min(seg_t0.values())
segs = sorted(seg_t0)
# seg_t0 is NOT usable to locate a segment: every segment's first logMonoTime is
# initData and they are all identical (verified on 23e -- all 42 report offset 0.0).
# logMonoTime is continuous across segments and each covers ~60 s, so derive the
# segment span from the window itself. SEGS=a,b overrides.
import os
_env = os.environ.get('SEGS')
if _env:
  want = [int(x) for x in _env.split(',')]
else:
  want = [s for s in segs if int(A // 60) - 1 <= s <= int(B // 60) + 1 and s >= 0]
print(f"# route {r} window {A:.2f}..{B:.2f} s  segments {want}")

ri, why = build_radar_interface('HONDA_CIVIC_BOSCH'); assert ri is not None, why
ids = set(RI.BOSCH_A_ALL_IDS)
ctx = {'v': None, 'a': None, 'la': None, 'l1': None, 'mp': None}
seen = {}          # tid -> [rt, ...]
rows = []

def sweep(t):
  rt = (t - T0) / 1e9
  vl = ri.rcp.vl; objs = []
  for slot in range(RI.BOSCH_A_NUM_SLOTS):
    f0, f1, f2, f3 = RI.BOSCH_A_MAIN_IDS[slot]
    v0, v1, v2, v3 = vl[f0], vl[f1], vl[f2], vl[f3]
    st, rr, ar = int(v0['STATUS']), int(v0['RANGE_RAW']), int(v0['AZIMUTH_RAW'])
    life, tid = int(v2['LIFECYCLE_RAW']), int(v3['TRACK_ID'])
    if not (st != RI.BOSCH_A_STATUS_INVALID and rr != RI.BOSCH_A_RANGE_RAW_INVALID
            and ar != RI.BOSCH_A_ANGLE_RAW_INVALID and life != RI.BOSCH_A_LIFE_INVALID
            and RI.BOSCH_A_TRACK_ID_MIN <= tid <= RI.BOSCH_A_TRACK_ID_MAX): continue
    dr = RI.BOSCH_A_RANGE_SCALE_M * rr + RI.BOSCH_A_RANGE_OFFSET_M
    yr = dr * math.tan(RI.BOSCH_A_AZIMUTH_SCALE_RAD * (ar - RI.BOSCH_A_AZIMUTH_CENTER))
    objs.append((tid, dr, yr, int(v1['OBJECT_EXISTENCE_PROBABILITY_RAW'])))
  for o in objs:
    seen.setdefault(o[0], []).append(rt)
  if A <= rt <= B:
    s = " ".join(f"[{o[0]} d{o[1]:.1f} y{o[2]:.1f} ex{o[3]}]" for o in sorted(objs, key=lambda z: z[1]))
    f = lambda x, n=2: (None if x is None else round(x, n))
    rows.append(f"{int(rt//60)}:{rt%60:05.2f} nvalid={len(objs):2d} vEgo={f(ctx['v'])} "
                f"accel={f(ctx['a'])} longActive={ctx['la']} lead1={ctx['l1']} mlp={f(ctx['mp'])} "
                f"| {s or '(NO VALID OBJECTS)'}")

pending, pending_t = [], None
for sg in want:
  g = glob.glob(f'/routes/{r}/{sg}/rlog*')
  if not g: print(f"# seg {sg} missing"); continue
  for m in _LogFileReader(g[0]):
    w = m.which()
    if w == 'can':
      t = m.logMonoTime
      if pending_t is not None and t != pending_t and pending:
        if ri.update([(pending_t, pending)]) is not None: sweep(pending_t)
        pending = []
      pending_t = t
      for fr in m.can:
        if fr.address in ids: pending.append(CanData(fr.address, bytes(fr.dat), fr.src))
    elif w == 'carState': ctx['v'] = m.carState.vEgo
    elif w == 'carControl':
      try: ctx['a'] = m.carControl.actuators.accel; ctx['la'] = m.carControl.longActive
      except Exception: pass
    elif w == 'radarState':
      l = m.radarState.leadOne
      ctx['l1'] = (l.radarTrackId, round(l.dRel, 1), round(l.vRel, 1)) if l.status else None
    elif w == 'modelV2':
      ls = m.modelV2.leadsV3; ctx['mp'] = ls[0].prob if len(ls) else None

for line in rows: print(line)
if TID is not None:
  ts = seen.get(TID)
  if ts:
    print(f"\ntrack {TID} present on {len(ts)} sweeps, rt {min(ts):.2f} .. {max(ts):.2f}")
    gaps = [(ts[i], ts[i+1]) for i in range(len(ts)-1) if ts[i+1]-ts[i] > 0.5]
    print(f"track {TID} gaps >0.5s:", [f"{a:.2f}->{b:.2f} ({b-a:.2f}s)" for a, b in gaps])
  else:
    print(f"\ntrack {TID} never seen in segments {want}")
