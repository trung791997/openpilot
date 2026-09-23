"""Census of NEAR IN-LANE TARGET DROPOUTS across the Bosch-A fleet.

usage: dropcensus.py <route> [min_gap_s]

Motivation (route 241, 9:36): the failure mode is NOT the sensor going blank. On 241 the
all-16-slots-invalid condition lasted 0.14 s, while the *near in-lane* target (track 20) was
absent for ~3.7 s as the radar happily kept reporting far, off-axis objects. A census keyed on
"all slots invalid" scores that event as a non-event and is dominated by empty road / parked.

So this counts a different thing: a stretch where the radar reports NO object in the near
in-lane box, having reported one immediately before, while the car is under longitudinal
control and commanding acceleration.

Per sweep an object is "near in-lane" iff d < NEAR_D and |y| < LANE_Y, using exactly the
rawslots.py validity predicate and the parser's own range/azimuth scaling. `blank_frac` reports
what fraction of the gap had NO valid objects at all -- that is the column that separates a
genuine sensor blackout from a selective dropout.
"""
import sys, json, glob, math, os
from collections import deque

sys.path.insert(0, "/src")
sys.path.insert(0, "/src/openpilot/tools")
from openpilot.tools.lib.logreader import _LogFileReader
from opendbc.car.can_definitions import CanData
import opendbc.car.honda.radar_interface as RI
from bosch_a_route_report import build_radar_interface

r = sys.argv[1]
MIN_GAP_S = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

NEAR_D = 40.0    # m   -- "near"; 241's track 20 sat at 23-26 m
LANE_Y = 2.0     # m   -- half-width of the in-lane box (~a lane either side of centre)
EX_MIN = 32      #     -- presence bar, deliberately low: we are counting disappearance
PRE_S  = 0.5     # s   -- the target must have been held this long before vanishing to count
SURVIVE_S = 1.5  # s   -- window in which a re-sighting outside the box counts as an exit
SURVIVE_MIN_S = 0.75  # s -- and it must still be there this long after, not just trailing off

# reporting filters: what makes a gap *dangerous* rather than merely empty road
ACCEL_MIN = 0.5  # m/s^2 commanded during the gap
VEGO_MIN  = 2.0  # m/s   -- moving, not stop-and-go creep

fp = "HONDA_CIVIC_BOSCH"
d = json.load(open(f"/routes/an2/scan_{r}.json"))
T0 = min(int(v) for v in d["seg_t0"].values())
ids = set(RI.BOSCH_A_ALL_IDS)
segs = sorted(int(os.path.basename(p)) for p in glob.glob(f"/routes/{r}/*")
              if os.path.basename(p).isdigit())

ctx = {"vEgo": None, "accel": None, "longActive": None,
       "lead_tid": None, "lead_d": None, "mlprob": None}

gaps = []
cur = None          # open gap
cov_run_start = None  # rt at which the current covered stretch began
last_cov = None     # (rt, tid, d, y, ex) of the last near in-lane sighting
hist = deque()      # recent (rt, d, y, ex) of the near in-lane object, ~2s window


def close_gap(rt, reason):
  global cur
  if cur is not None:
    cur["t1"] = rt
    cur["end"] = reason
    gaps.append(cur)
    cur = None


MAX_DT = 2.0   # s -- a larger jump between sweeps is a data hole, not a dropout
last_rt = None


def sweep(rt):
  global cur, cov_run_start, last_cov, last_rt
  # A hole in the recording must never masquerade as the radar losing a target.
  # logMonoTime is continuous ACROSS segments, so an ordinary segment boundary is
  # not a discontinuity and a real dropout is allowed to span it.
  if last_rt is not None and (rt - last_rt) > MAX_DT:
    cur = None
    cov_run_start = None
    last_cov = None
  last_rt = rt

  vl = RI_ri.rcp.vl
  objs = []
  for slot in range(RI.BOSCH_A_NUM_SLOTS):
    f0, f1, f2, f3 = RI.BOSCH_A_MAIN_IDS[slot]
    v0, v1, v2, v3 = vl[f0], vl[f1], vl[f2], vl[f3]
    st, rr, ar = int(v0['STATUS']), int(v0['RANGE_RAW']), int(v0['AZIMUTH_RAW'])
    life, tid = int(v2['LIFECYCLE_RAW']), int(v3['TRACK_ID'])
    if not (st != RI.BOSCH_A_STATUS_INVALID and rr != RI.BOSCH_A_RANGE_RAW_INVALID
            and ar != RI.BOSCH_A_ANGLE_RAW_INVALID and life != RI.BOSCH_A_LIFE_INVALID
            and RI.BOSCH_A_TRACK_ID_MIN <= tid <= RI.BOSCH_A_TRACK_ID_MAX):
      continue
    dr = RI.BOSCH_A_RANGE_SCALE_M * rr + RI.BOSCH_A_RANGE_OFFSET_M
    yr = dr * math.tan(RI.BOSCH_A_AZIMUTH_SCALE_RAD * (ar - RI.BOSCH_A_AZIMUTH_CENTER))
    ex = int(v1['OBJECT_EXISTENCE_PROBABILITY_RAW'])
    objs.append((tid, dr, yr, ex))

  inlane = [o for o in objs if o[1] < NEAR_D and abs(o[2]) < LANE_Y and o[3] >= EX_MIN]

  if inlane:
    near = min(inlane, key=lambda z: z[1])
    last_cov = (rt, near[0], near[1], near[2], near[3])
    if cov_run_start is None:
      cov_run_start = rt
      hist.clear()
    hist.append((rt, near[1], near[2], near[3], near[0]))
    while hist and rt - hist[0][0] > 2.0:
      hist.popleft()
    close_gap(rt, "reacquired")
    return

  # no near in-lane object this sweep
  if cur is None:
    held = None if cov_run_start is None else (last_cov[0] - cov_run_start)
    cov_run_start = None
    if last_cov is None or held is None or held < PRE_S:
      return          # nothing credible was there to lose
    # How did the target leave? Geometry alone cannot tell a lead that drifted out of the
    # box from one that vanished: on 241 track 20's azimuth wandered to the lane edge as its
    # return weakened, which *looks* like a lateral exit. The honest test is survival -- does
    # that track id still appear ANYWHERE in the object list just after it left the box? If it
    # does, it exited. If the whole track evaporates, the radar lost it. Decided below, once
    # the following sweeps have been seen.
    d_last, y_last, ex_last = last_cov[2], last_cov[3], last_cov[4]
    d_rate = 0.0
    ex_max = ex_last
    # Use ONLY the trailing run of samples carrying the SAME track id. `hist` holds the
    # nearest in-lane object, which can switch tracks mid-window and then yields a rate
    # that is not any object's rate (the -29.7 / -30.9 m/s values seen in the first pass).
    run = []
    for h in reversed(hist):
      if h[4] != last_cov[1]: break
      run.append(h)
    run.reverse()
    if len(run) >= 2:
      dt = run[-1][0] - run[0][0]
      if dt > 0.2:
        d_rate = (run[-1][1] - run[0][1]) / dt
      ex_max = max(h[3] for h in run)
    kind = "DROPOUT"
    # ONCOMING TRAFFIC IS NOT A DROPOUT. An oncoming car crosses the boresight, sweeps the
    # in-lane box for ~1 s and passes out of the radar's near field at ~10 m. It dies at full
    # existence with no fade, which is exactly the "no warning" signature -- and it accounted
    # for BOTH such cases in the first pass (232 10:05 track 1, 23e 13:38 track 9: range
    # collapsing at ~29 m/s while ego did 14-16 m/s, y sweeping -4.6 -> +2.8 as it went by).
    # Nothing ahead can close faster than ego speed unless it is coming the other way.
    if ctx["vEgo"] is not None and d_rate < -(ctx["vEgo"] + 3.0):
      kind = "oncoming"
    cur = {"t0": rt, "t1": rt, "n": 0,
           "before": last_cov, "held": held,
           "kind": kind, "d_rate": d_rate, "ex_max": ex_max,
           "lost_tid": last_cov[1], "survived_at": None,
           "accel_max": None, "accel_5s": None, "vEgo_max": None, "vEgo_at0": ctx["vEgo"],
           "mlprob_min": None, "longActive": False,
           # AT THE MOMENT OF LOSS. `longActive` below is an OR over the whole gap and
           # `accel_5s` a max over its first 5 s, so the two can come from different
           # moments -- on 23e 23:46 the track died while longitudinal control was OFF
           # (driver had disengaged 2.5 s earlier) and the 1.14 m/s^2 came from after a
           # re-engagement 2 s later. Reporting those together read as "the car
           # accelerated into a lead it had lost", which is not what happened.
           "la_at0": ctx["longActive"], "accel_at0": ctx["accel"],
           "n_eng": 0,
           "n_blank": 0, "nvalid_min": 99, "nvalid_max": 0,
           "lead_lost": None, "end": "eof"}

  cur["t1"] = rt
  cur["n"] += 1
  # survival window: the lost track reappearing outside the box means it exited, not dropped out
  # Must be SUSTAINED: track 20 on 241 was still in the list for 0.07 s after its azimuth
  # crossed the box edge, which a naive "seen at all" test scores as a clean lateral exit.
  if cur["kind"] == "DROPOUT" and SURVIVE_MIN_S <= (rt - cur["t0"]) <= SURVIVE_S:
    for o in objs:
      if o[0] == cur["lost_tid"]:
        cur["kind"] = "exited"
        cur["survived_at"] = (o[1], o[2])
        break
  if not objs:
    cur["n_blank"] += 1
  cur["nvalid_min"] = min(cur["nvalid_min"], len(objs))
  cur["nvalid_max"] = max(cur["nvalid_max"], len(objs))
  a, v, p = ctx["accel"], ctx["vEgo"], ctx["mlprob"]
  if a is not None:
    cur["accel_max"] = a if cur["accel_max"] is None else max(cur["accel_max"], a)
    if rt - cur["t0"] <= 5.0:
      cur["accel_5s"] = a if cur["accel_5s"] is None else max(cur["accel_5s"], a)
  if v is not None:
    cur["vEgo_max"] = v if cur["vEgo_max"] is None else max(cur["vEgo_max"], v)
  if p is not None:
    cur["mlprob_min"] = p if cur["mlprob_min"] is None else min(cur["mlprob_min"], p)
  if ctx["longActive"]:
    cur["longActive"] = True
    cur["n_eng"] += 1
  if cur["lead_lost"] is None and ctx["lead_tid"] is None:
    cur["lead_lost"] = rt


RI_ri, why = build_radar_interface(fp)
assert RI_ri is not None, why

for seg in segs:
  g = glob.glob(f"/routes/{r}/{seg}/rlog*")
  if not g:
    continue
  pending, pending_t = [], None
  try:
    for m in _LogFileReader(g[0]):
      w = m.which()
      if w == "can":
        t = m.logMonoTime
        if pending_t is not None and t != pending_t and pending:
          if RI_ri.update([(pending_t, pending)]) is not None:
            rt = (pending_t - T0) / 1e9
            if rt >= 0:
              sweep(rt)
          pending = []
        pending_t = t
        for f in m.can:
          if f.address in ids:
            pending.append(CanData(f.address, bytes(f.dat), f.src))
      elif w == "carState":
        ctx["vEgo"] = m.carState.vEgo
      elif w == "carControl":
        try:
          ctx["accel"] = m.carControl.actuators.accel
          ctx["longActive"] = m.carControl.longActive
        except Exception:
          pass
      elif w == "radarState":
        l1 = m.radarState.leadOne
        ctx["lead_tid"] = l1.radarTrackId if l1.status else None
        ctx["lead_d"] = l1.dRel if l1.status else None
      elif w == "modelV2":
        ls = m.modelV2.leadsV3
        ctx["mlprob"] = ls[0].prob if len(ls) else None
  except Exception as e:
    print(f"  !! {r} seg {seg}: {type(e).__name__}: {e}", file=sys.stderr)

close_gap(cur["t1"] if cur else 0.0, "eof")


def f(v, p="%.2f"):
  return "na" if v is None else (p % v)


long_enough = [x for x in gaps if (x["t1"] - x["t0"]) >= MIN_GAP_S]
for x in gaps:
  x["dur"] = x["t1"] - x["t0"]
  x["score"] = min(x["dur"], 10.0) * (x["accel_5s"] or 0.0)
# `la_at0` is the hard requirement, not `longActive`: the car must have been under
# longitudinal control AT THE MOMENT the near in-lane target was lost. An OR over the
# whole gap admits events where the driver was already driving manually when the radar
# dropped the lead, which is a different (and far less alarming) situation.
danger = [x for x in long_enough
          if x["la_at0"] and (x["accel_5s"] or -9) >= ACCEL_MIN
          and (x["vEgo_max"] or 0) >= VEGO_MIN and x["kind"] == "DROPOUT"]
# Kept separately so the correction is visible rather than silently dropping rows.
demoted = [x for x in long_enough
           if (not x["la_at0"]) and x["longActive"] and (x["accel_5s"] or -9) >= ACCEL_MIN
           and (x["vEgo_max"] or 0) >= VEGO_MIN and x["kind"] == "DROPOUT"]

if os.environ.get("DUMPALL"):
  print("--- all gaps >= %.1fs (pre-filter) ---" % MIN_GAP_S)
  for x in sorted(long_enough, key=lambda z: z["t0"]):
    print(f"  {int(x['t0']//60)}:{x['t0']%60:05.2f} dur={x['t1']-x['t0']:6.2f}s "
          f"la={x['longActive']} accelMax={f(x['accel_max'])} vEgoMax={f(x['vEgo_max'])} "
          f"lost[tid={x['before'][1]} d={x['before'][2]:.1f} ex={x['before'][4]}] "
          f"kind={x['kind']} drate={x['d_rate']:+.1f}"
          + (f" survived@d={x['survived_at'][0]:.1f},y={x['survived_at'][1]:+.1f}" if x['survived_at'] else "")
          + f" end={x['end']}")

kinds = {}
for x in long_enough:
  kinds[x["kind"]] = kinds.get(x["kind"], 0) + 1
print(f"### {r}: {len(segs)} segs | in-lane gaps {len(gaps)} total, {len(long_enough)} >={MIN_GAP_S}s "
      f"{kinds} | {len(danger)} DROPOUT + engaged + accelerating")
for x in sorted(danger, key=lambda z: -z["score"]):
  b = x["before"]
  blank = 100.0 * x["n_blank"] / max(1, x["n"])
  fade = "" if not x["ex_max"] else f" exFade={b[4]}/{x['ex_max']}"
  print(f"  {int(x['t0']//60)}:{x['t0']%60:05.2f} dur={x['dur']:6.2f}s score={x['score']:5.2f} "
        f"lost[tid={b[1]} d={b[2]:.1f} y={b[3]:+.1f} ex={b[4]} held={x['held']:.1f}s"
        f" drate={x['d_rate']:+.1f}{fade}] "
        f"accel5s={f(x['accel_5s'])} accelMax={f(x['accel_max'])} "
        f"vEgo={f(x['vEgo_at0'])}->{f(x['vEgo_max'])} "
        f"blank={blank:4.1f}% nvalid[{x['nvalid_min']},{x['nvalid_max']}] "
        f"mlprobMin={f(x['mlprob_min'])} "
        f"laAt0={x['la_at0']} accelAt0={f(x['accel_at0'])} eng={100.0*x['n_eng']/max(1,x['n']):.0f}%")

for x in sorted(demoted, key=lambda z: -z["score"]):
  b = x["before"]
  print(f"  DEMOTED(not engaged at loss) {int(x['t0']//60)}:{x['t0']%60:05.2f} "
        f"dur={x['dur']:6.2f}s score={x['score']:5.2f} tid={b[1]} d={b[2]:.1f} "
        f"accel5s={f(x['accel_5s'])} accelAt0={f(x['accel_at0'])} "
        f"eng={100.0*x['n_eng']/max(1,x['n']):.0f}%")
